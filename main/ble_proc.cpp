#include <esp_bt.h>
#include <esp_log.h>
#include <esp_mac.h>

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "adc_ble_interface.h"
#include "adc_proc.h"
#include "ble_ota_interface.h"
#include "ble_proc.h"
#include "dynamite_uuid.h"
#include "user_kvs.h"

#include "board_id.h"
#include "build_metadata.h"

#include <string.h>

constexpr char TAG[] = "BLE";

static_assert(CONFIG_BT_NIMBLE_MAX_CONNECTIONS == 1);

struct AppGlobal {
	uint16_t connHandle;
	uint16_t valHandle;
};

static AppGlobal appGlobalConn;

StreamBufferHandle_t adcStreamBufferHandle = NULL;
static size_t adcFeedSamplesPerChunk = 0;

DeviceLock deviceLock = DeviceLock::Open;

struct DleConnInfo {
	uint16_t maxTxOctets;
	uint16_t maxRxOctets;
	uint16_t maxTxTime;
	uint16_t maxRxTime;
	bool negotiated;
};

constexpr DleConnInfo dleInfoDefaults{
    .maxTxOctets = 0x001B, // BLE defaults: 27 octets
    .maxRxOctets = 0x001B,
    .maxTxTime = 0x0148, // 328 us (units are 8 us)
    .maxRxTime = 0x0148,
    .negotiated = false,
};

static DleConnInfo dleConnInfo{dleInfoDefaults};

static int gapEventHandler(ble_gap_event *event, void *arg);

constexpr esp_power_level_t dbmToEspPowerLevel(int dbm) {
	// +20 dBm is a special case, not another 3 dB step.
	if (dbm == 20) {
		return ESP_PWR_LVL_P20;
	}
	// Valid regular settings: -12 through +18 dBm, in 3 dB steps.
	if (dbm < -12 || dbm > 18 || ((dbm + 12) % 3) != 0) {
		return ESP_PWR_LVL_INVALID;
	}
	return (esp_power_level_t)((int)ESP_PWR_LVL_N12 + (dbm + 12) / 3);
}

constexpr int8_t espPowerLevelToDbm(esp_power_level_t level_) {
	const int level = (int)level_;
	if (level == (int)ESP_PWR_LVL_P20) {
		return 20;
	}
	if (level >= (int)ESP_PWR_LVL_N12 && level <= (int)ESP_PWR_LVL_P18) {
		return -12 + 3 * (level - (int)ESP_PWR_LVL_N12);
	}
	return 0;
}

static int dleGapListener(ble_gap_event *event, void *arg) {
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		if (event->connect.status == 0) {
			// New connection: forget the previous peer's negotiated DLE.
			// If this peer supports DLE, DATA_LEN_CHG will overwrite these shortly.
			dleConnInfo = dleInfoDefaults;
		}
		break;

	case BLE_GAP_EVENT_DISCONNECT:
		dleConnInfo = dleInfoDefaults; // no stale values while disconnected
		break;

	case BLE_GAP_EVENT_DATA_LEN_CHG:
		dleConnInfo.maxTxOctets = event->data_len_chg.max_tx_octets;
		dleConnInfo.maxRxOctets = event->data_len_chg.max_rx_octets;
		dleConnInfo.maxTxTime = event->data_len_chg.max_tx_time;
		dleConnInfo.maxRxTime = event->data_len_chg.max_rx_time;
		dleConnInfo.negotiated = true;
		ESP_LOGD(TAG, "Gap event data len chg octets Tx,Rx (%u, %u), time Tx,Rx (%u,%u)",
		         dleConnInfo.maxTxOctets, dleConnInfo.maxRxOctets, dleConnInfo.maxTxTime,
		         dleConnInfo.maxRxTime);
		break;

	default:
		break;
	}
	return 0;
}

static void adcOnDisconnect() {
	if (deviceLock != DeviceLock::Streaming) {
		return;
	}
	stopAdcAcquisition();
	deviceLock = DeviceLock::Open;
}

static void adcOnSubscr(bool subscr) {
	if (subscr) {
		if (deviceLock == DeviceLock::Open) {
			deviceLock = DeviceLock::Streaming;

			adcFeedSamplesPerChunk = (dleConnInfo.maxTxOctets - L2CAP_HEADER_SIZE -
			                          ATT_HEADER_SIZE - sizeof(AdcFeedNetworkPacket::Header)) /
			                         sizeof(AdcFeedNetworkData);
			if (adcFeedSamplesPerChunk > AdcFeedNetworkPacket::MAX_NUM_SAMPLES) {
				adcFeedSamplesPerChunk = AdcFeedNetworkPacket::MAX_NUM_SAMPLES;
			}
			ESP_LOGI(TAG, "Samples per chunk %u", adcFeedSamplesPerChunk);
			if (!startAdcAcquisition(adcFeedSamplesPerChunk)) {
				deviceLock = DeviceLock::Open;
			}
		}
	} else {
		adcOnDisconnect();
	}
}

constexpr size_t HW_ID_LEN = 12; // NOT zero terminated

static void hwIdStr(char out[HW_ID_LEN]) {
	uint8_t mac[8];
	esp_efuse_mac_get_default(mac);
	static constexpr char hexChr[] = "0123456789ABCDEF";
	for (size_t i = 0; i < 6; i++) {
		uint8_t byte = mac[i];
		out[i * 2] = hexChr[byte >> 4];
		out[i * 2 + 1] = hexChr[byte & 0x0F];
	}
}

static void startAdvertising(void *context) {
	ble_hs_adv_fields fields{};
	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	fields.uuids128 = &DYNAMITE_SAMPLER_SVC_UUID128;
	fields.num_uuids128 = 1;
	fields.uuids128_is_complete = 1;
	ble_gap_adv_set_fields(&fields);

	const char *name = ble_svc_gap_device_name();
	ble_hs_adv_fields rsp{};
	rsp.name = (uint8_t *)name;
	rsp.name_len = strlen(name);
	rsp.name_is_complete = 1;
	ble_gap_adv_rsp_set_fields(&rsp);

	static constexpr ble_gap_adv_params advParams{
	    .conn_mode = BLE_GAP_CONN_MODE_UND,
	    .disc_mode = BLE_GAP_DISC_MODE_GEN,
	    .itvl_min = 0,
	    .itvl_max = 0,
	    .channel_map = 0,
	    .filter_policy = BLE_HCI_ADV_FILT_NONE,
	    .high_duty_cycle = 0,
	};
	ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER, &advParams, gapEventHandler,
	                  context);
}

static int gapEventHandler(ble_gap_event *event, void *arg) {
	AppGlobal *appContext = (AppGlobal *)arg;
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT: {
		ESP_LOGI(TAG, "connHandle=%u", event->connect.conn_handle);
		if (event->connect.status) {
			appContext->connHandle = 0;
			startAdvertising(appContext);
		} else {
			appContext->connHandle = event->connect.conn_handle;
			// TODO: Figure out:
			// is this needed?
			// Does this actually affect the packet size for the way we notify
			// Do we need to notify in a different way?
			ble_gap_set_data_len(event->connect.conn_handle, DLE_MAX_SIZE, (DLE_MAX_SIZE + 14) * 8);

			// Best case, devices can handle 7.5ms interval connection (android phone)
			// older iOS might be 15ms, Windows PC might be 30ms. Connection interval is set in
			// increments of 1.25ms.
			// Don't skip any connection intervals, and timout after 10*N ms
			static constexpr ble_gap_upd_params params = {
			    .itvl_min = 6,
			    .itvl_max = 6 * 4,
			    .latency = 0,
			    .supervision_timeout = 500,
			    .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
			    .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
			};
			ble_gap_update_params(event->connect.conn_handle, &params);
		}
	} break;
	case BLE_GAP_EVENT_DISCONNECT: {
		adcOnDisconnect();
		otaOnDisconnect();
		appContext->connHandle = 0;
		startAdvertising(appContext);
		ESP_LOGD(TAG, "Server onDisco, core %u", xPortGetCoreID());
	} break;
	case BLE_GAP_EVENT_ADV_COMPLETE: {
		startAdvertising(appContext);
	} break;
	case BLE_GAP_EVENT_SUBSCRIBE: {
		ESP_LOGI(TAG, "subscr handles: %u %u", event->subscribe.conn_handle,
		         event->subscribe.attr_handle);
		if (!boardCfg()) {
			ESP_LOGW(TAG, "ADC not operational");
			break;
		}
		if (appContext->valHandle != event->subscribe.attr_handle) {
			break;
		}
		adcOnSubscr(event->subscribe.cur_notify);
	} break;
	case BLE_GAP_EVENT_MTU: {
		ESP_LOGI(TAG, "mtu=%u", event->mtu.value);
	} break;
	case BLE_GAP_EVENT_DATA_LEN_CHG: {
		ESP_LOGI(TAG, "rxo=%u txo=%u", event->data_len_chg.max_rx_octets,
		         event->data_len_chg.max_tx_octets);
	} break;
	default:
		break;
	}
	return 0;
}

// Task that is notified when the ADC buffer is ready to be sent
static void IRAM_ATTR taskBlePublishAdcBuffer(void *) {
	uint16_t count = 0;
	while (true) {
		AdcFeedNetworkPacket packet;
		static_assert(sizeof(packet) <= ATT_PAYLOAD_MAX_SIZE);
		// Read the ADC buffer and update the BLE characteristic
		size_t bytesRead = xStreamBufferReceive(adcStreamBufferHandle, &packet.adc,
		                                        sizeof(packet.adc), portMAX_DELAY);
		if (bytesRead == sizeof(packet.adc)) [[likely]] {
			packet.hdr.sample_sequence_number = htole16(count);
			count += adcFeedSamplesPerChunk;

			os_mbuf *om = ble_hs_mbuf_from_flat(
			    &packet, sizeof(packet.hdr) + adcFeedSamplesPerChunk * sizeof(*packet.adc));
			if (om) [[likely]] {
				ble_gatts_notify_custom(appGlobalConn.connHandle, appGlobalConn.valHandle, om);
			}
		}
	}
	vTaskDelete(NULL);
}

static void onSync() {
	startAdvertising(&appGlobalConn);
	ESP_LOGI(TAG, "Advertising started");
}

static void bleHostTask(void *) {
	nimble_port_run(); // This function will return only when nimble_port_stop() is executed
	                   // nimble_port_deinit();
}

consteval ble_gatt_svc_def defineService(const ble_uuid_t *uuid_, const ble_gatt_chr_def *chr_) {
	return ble_gatt_svc_def{
	    .type = BLE_GATT_SVC_TYPE_PRIMARY,
	    .uuid = uuid_,
	    .includes = nullptr,
	    .characteristics = chr_,
	};
}

consteval ble_gatt_chr_def defineCharacteristic(const ble_uuid_t *uuid_,
                                                ble_gatt_access_fn *access_cb_, void *arg_,
                                                ble_gatt_chr_flags flags_, uint16_t *val_handle_) {
	return ble_gatt_chr_def{
	    .uuid = uuid_,
	    .access_cb = access_cb_,
	    .arg = arg_,
	    .descriptors = nullptr,
	    .flags = flags_,
	    .min_key_size = 0,
	    .val_handle = val_handle_,
	    .cpfd = nullptr,
	};
}

static int onMakeName(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                      void *arg) {
	return os_mbuf_append(ctxt->om, DEVICE_MANUFACTURER_NAME, sizeof(DEVICE_MANUFACTURER_NAME) - 1)
	           ? BLE_ATT_ERR_INSUFFICIENT_RES
			   : 0;
}

static int onModelNum(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                      void *arg) {
	const char *s = boardCfg() ? boardCfg()->marketingName : BOARD_IDENTITY_MARKETING_NONE;
	return os_mbuf_append(ctxt->om, s, strlen(s)) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
}

static int onSerialNum(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                       void *arg) {
	char hwId[HW_ID_LEN + 1]{0};
	hwIdStr(hwId);
	return os_mbuf_append(ctxt->om, hwId, HW_ID_LEN) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
}

static int onHwRev(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                   void *arg) {
	const char *s = boardCfg() ? boardCfg()->name : BOARD_IDENTITY_NONE;
	return os_mbuf_append(ctxt->om, s, strlen(s)) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
}

static int onFwVer(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                   void *arg) {
	return os_mbuf_append(ctxt->om, GIT_DESCRIBE, sizeof(GIT_DESCRIBE) - 1)
	           ? BLE_ATT_ERR_INSUFFICIENT_RES
			   : 0;
}

static int onTxPowerRead(uint16_t connHandle, uint16_t attrHandle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
	TxPowerNetworkData rPwr{
	    .val = espPowerLevelToDbm(esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT)),
	};
	return os_mbuf_append(ctxt->om, &rPwr, sizeof(rPwr)) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
}

static int onTxPowerWrite(uint16_t connHandle, uint16_t attrHandle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
	TxPowerNetworkData rPwr;
	if (OS_MBUF_PKTLEN(ctxt->om) == sizeof(rPwr)) {
		os_mbuf_copydata(ctxt->om, 0, sizeof(rPwr), &rPwr);
		esp_power_level_t v = dbmToEspPowerLevel(rPwr.val);
		if (v != ESP_PWR_LVL_INVALID) {
			esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, v);
		}
	}
	return 0;
}

static int onKvs(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt,
                 void *arg) {
	char buff[USER_KVS_NETWORK_FRAME_LENGTH + 1]{0};
	const size_t rqLength = OS_MBUF_PKTLEN(ctxt->om);
	if (rqLength + 2 < sizeof(buff)) {
		os_mbuf_copydata(ctxt->om, 0, rqLength, buff + 1);
	}
	ESP_LOGI(TAG, "Rq '%s'", buff + 1);
	char status = KvsStatusBusy;
	if (deviceLock == DeviceLock::Open) {
		switch (processKvsCommand(buff + 1, rqLength, buff + (rqLength + 2),
		                          sizeof(buff) - (rqLength + 2))) {
		case KvsResult::Ok:
			status = KvsStatusOk;
			buff[rqLength + 1] = '=';
			break;
		case KvsResult::Rejected:
			status = KvsStatusRejected;
			break;
		case KvsResult::Error:
			status = KvsStatusError;
			break;
		}
	} else {
		ESP_LOGW(TAG, "KVS command: Device locked(%u)", deviceLock);
	}
	buff[0] = status;
	ESP_LOGI(TAG, "Repl '%s'", buff);

	os_mbuf *om = ble_hs_mbuf_from_flat(buff, strlen(buff));
	if (om) {
		ble_gatts_notify_custom(connHandle, attrHandle, om);
	}
	return 0;
}

static int onAdcConfig(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt,
                       void *arg) {
	AdcConfigNetworkData data{getAdcConfig()};
	return os_mbuf_append(ctxt->om, &data, sizeof(data)) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
}

static int dummyAccessCb(uint16_t connHandle, uint16_t attrHandle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
	return 0;
}

constexpr ble_gatt_chr_def deviceInfoChr[] = {
    defineCharacteristic(&DEVICE_MAKE_NAME_CHR_UUID16.u, onMakeName, nullptr, BLE_GATT_CHR_F_READ,
	                     nullptr),
    defineCharacteristic(&DEVICE_MODEL_NUMBER_CHR_UUID16.u, onModelNum, nullptr,
	                     BLE_GATT_CHR_F_READ, nullptr),
    defineCharacteristic(&DEVICE_SERIAL_NUMBER_CHR_UUID16.u, onSerialNum, nullptr,
	                     BLE_GATT_CHR_F_READ, nullptr),
    defineCharacteristic(&DEVICE_HARDWARE_REV_CHR_UUID16.u, onHwRev, nullptr, BLE_GATT_CHR_F_READ,
	                     nullptr),
    defineCharacteristic(&DEVICE_FIRMWARE_VER_CHR_UUID16.u, onFwVer, nullptr, BLE_GATT_CHR_F_READ,
	                     nullptr),
    defineCharacteristic(&DEVICE_TX_POWER_CHR_UUID16.u, onTxPowerRead, nullptr, BLE_GATT_CHR_F_READ,
	                     nullptr),
    {},
};

constexpr ble_gatt_chr_def adcNotifyChr[] = {
    defineCharacteristic(&ADC_FEED_CHR_UUID128.u, dummyAccessCb, nullptr, BLE_GATT_CHR_F_NOTIFY,
	                     &appGlobalConn.valHandle),
    defineCharacteristic(&USER_KVS_CHR_UUID128.u, onKvs, nullptr,
	                     BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE, nullptr),
    defineCharacteristic(&ADC_CONF_CHR_UUID128.u, onAdcConfig, nullptr, BLE_GATT_CHR_F_READ,
	                     nullptr),
    {},
};

constexpr ble_gatt_chr_def txPowerChr[] = {
    defineCharacteristic(&TX_PWR_CHR_UUID128.u, onTxPowerWrite, nullptr, BLE_GATT_CHR_F_WRITE,
	                     nullptr),
    {},
};

constexpr ble_gatt_svc_def gattSvcsDefinition[] = {
    defineService(&DEVICE_INFO_SVC_UUID16.u, deviceInfoChr),
    defineService(&DYNAMITE_SAMPLER_SVC_UUID128.u, adcNotifyChr),
    defineService(&TX_PWR_SVC_UUID128.u, txPowerChr),
    // defineService(&OTA_SVC_UUID, otaGattChr),
    {},
};

static void _taskSetupBle(void *setupDone) {
	ESP_LOGI(TAG, "Setting up BLE");
	// Create the BLE Device
	// Name the device with the mac address to make it unique for testing purposes.
	char bleName[CONFIG_BT_NIMBLE_GAP_DEVICE_NAME_MAX_LEN + 1] = {"DS "};
	static_assert(sizeof(bleName) >= 3 + HW_ID_LEN + 1);
	hwIdStr(bleName + 3);
	nimble_port_init();                   // 3 - Initialize the host stack
	ble_svc_gap_device_name_set(bleName); // 4 - Initialize NimBLE configuration - server name
	ble_svc_gap_init();                   // 4 - Initialize NimBLE configuration - gap service
	ble_svc_gatt_init();                  // 4 - Initialize NimBLE configuration - gatt service
	ble_gatts_count_cfg(
	    gattSvcsDefinition); // 4 - Initialize NimBLE configuration - config gatt services
	ble_gatts_add_svcs(
	    gattSvcsDefinition); // 4 - Initialize NimBLE configuration - queues gatt services.
	ble_att_set_preferred_mtu(BLE_ATT_MTU_MAX);
	static ble_gap_event_listener gapListener;
	ble_gap_event_listener_register(&gapListener, dleGapListener, nullptr);

	ble_hs_cfg.sync_cb = onSync;            // 5 - Initialize application
	nimble_port_freertos_init(bleHostTask); // 6 - Run the thread

	ESP_LOGI(TAG, "Setup done");
	ESP_LOGI(TAG, "Setup stack HWM %u", uxTaskGetStackHighWaterMark(NULL));

	*(volatile bool *)setupDone = true;
	vTaskDelete(NULL);
}

void setupBle(int core) {
	// Buffer to pass the ADC values from the ADC task to BLE task
	adcStreamBufferHandle = xStreamBufferCreate(ADC_FEED_MAX_CHUNK_SZ * 8, ADC_FEED_MAX_CHUNK_SZ);
	assert(adcStreamBufferHandle != NULL);

	volatile bool done = false;
	xTaskCreatePinnedToCore(_taskSetupBle, "task_BLE_setup", 1024 * 6, (void *)&done, 1, NULL,
	                        core);
	while (!done)
		vTaskDelay(10);

	// TODO figure out priority for the BLE task
	xTaskCreatePinnedToCore(taskBlePublishAdcBuffer, "task_BLE_publish", 1024 * 5, NULL, 3, NULL,
	                        core);

	ESP_LOGI(TAG, "Waiting a client connection to notify...");
}
