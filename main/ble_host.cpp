#include <esp_bt.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>

#include <cstdio>
#include <cstring>

#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_hs_adv.h>
#include <host/ble_hs_mbuf.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#include "adc_ble_interface.h"
#include "adc_proc.h"
#include "ble_host.h"
#include "dynamite_sampler_api.h"
#include "dynamite_uuid.h"
#include "user_kvs.h"

#include "board_id.h"
#include "build_metadata.h"

constexpr char TAG[] = "BLEH";

// GATT access runs only on the NimBLE host task, so one staging buffer per
// direction serves every characteristic and stays off that task's stack.
static uint8_t rxBuf[ATT_PAYLOAD_MAX_SIZE]; // client -> us
static uint8_t txBuf[ATT_PAYLOAD_MAX_SIZE]; // us -> client

//======================= Characteristic state

// Returns the length written to out, or 0 to refuse the read.
using ChrRead = size_t (*)(uint8_t *out, size_t cap);
// Returns the reply length to notify back, 0 to accept silently, or a
// negative value to refuse the write.
using ChrWrite = int (*)(const uint8_t *data, size_t len, uint8_t *reply, size_t replyCap);

// One per characteristic: the GATT definition's `arg`, so dispatch reaches it
// in one hop. The value handle lands here at registration; nothing hardcodes
// a handle number.
struct Chr {
	uint16_t valHandle = 0;
	ChrRead read = nullptr;   // null -> no READ flag
	ChrWrite write = nullptr; // null -> no WRITE flag
	uint16_t maxLen = 0;      // longer writes are refused before `write` runs
	bool notify = false;
};

// The GATT flags are derived from the hooks, so the table cannot disagree
// with them. A characteristic the dispatch callback rejects on flags alone
// never reaches the access callback.
static ble_gatt_chr_flags chrFlags(const Chr &chr) {
	return (chr.read ? BLE_GATT_CHR_F_READ : 0) | (chr.write ? BLE_GATT_CHR_F_WRITE : 0) |
	       (chr.notify ? BLE_GATT_CHR_F_NOTIFY : 0);
}

//======================= Device information

// The eFuse MAC as uppercase hex, not zero terminated.
constexpr size_t HW_ID_LEN = 12;
static char deviceSerial[HW_ID_LEN + 1];
static char deviceFirmware[BOARD_IDENTITY_NAME_MAX + sizeof(GIT_DESCRIBE)];

static void formatHwId() {
	uint8_t mac[8];
	ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
	static constexpr char HEX[] = "0123456789ABCDEF";
	for (size_t i = 0; i < HW_ID_LEN / 2; ++i) {
		deviceSerial[i * 2] = HEX[mac[i] >> 4];
		deviceSerial[i * 2 + 1] = HEX[mac[i] & 0x0F];
	}
}

static size_t emitString(const char *s, uint8_t *out, size_t cap) {
	const size_t len = strlen(s);
	if (len > cap) {
		return 0;
	}
	memcpy(out, s, len);
	return len;
}

static size_t readManufacturer(uint8_t *out, size_t cap) {
	return emitString(DEVICE_MANUFACTURER_NAME, out, cap);
}

static size_t readModel(uint8_t *out, size_t cap) {
	return emitString(boardCfg() ? boardCfg()->marketingName : BOARD_IDENTITY_MARKETING_NONE, out,
	                  cap);
}

static size_t readHwRevision(uint8_t *out, size_t cap) {
	return emitString(boardCfg() ? boardCfg()->name : BOARD_IDENTITY_NONE, out, cap);
}

static size_t readFirmware(uint8_t *out, size_t cap) {
	return emitString(deviceFirmware, out, cap);
}

static size_t readSerial(uint8_t *out, size_t cap) {
	if (HW_ID_LEN > cap) {
		return 0;
	}
	memcpy(out, deviceSerial, HW_ID_LEN);
	return HW_ID_LEN;
}

//======================= TX power

// The chip's power raster: 3 dBm steps up to P15, then P18 and P20. A dBm
// value off the raster is refused rather than rounded.
constexpr struct {
	esp_power_level_t level;
	int8_t dbm;
} TX_POWER_STEPS[] = {
    {ESP_PWR_LVL_N24, -24}, {ESP_PWR_LVL_N21, -21}, {ESP_PWR_LVL_N18, -18}, {ESP_PWR_LVL_N15, -15},
    {ESP_PWR_LVL_N12, -12}, {ESP_PWR_LVL_N9, -9},   {ESP_PWR_LVL_N6, -6},   {ESP_PWR_LVL_N3, -3},
    {ESP_PWR_LVL_N0, 0},    {ESP_PWR_LVL_P3, 3},    {ESP_PWR_LVL_P6, 6},    {ESP_PWR_LVL_P9, 9},
    {ESP_PWR_LVL_P12, 12},  {ESP_PWR_LVL_P15, 15},  {ESP_PWR_LVL_P18, 18},  {ESP_PWR_LVL_P20, 20},
};

// Anchors to the chip header: if another chip's enum layout ever lands here,
// these stop compiling instead of silently mis-mapping dBm the way
// "(level - N0) * 3" arithmetic does for the steps above P15.
static_assert(ESP_PWR_LVL_N24 == 0);
static_assert(ESP_PWR_LVL_N0 == 8);
static_assert(ESP_PWR_LVL_P15 == 13);
static_assert(ESP_PWR_LVL_P20 == 15);

static_assert(sizeof(TxPowerNetworkData) == 1);

static size_t readTxPower(uint8_t *out, size_t cap) {
	(void)cap;
	const esp_power_level_t level = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
	for (const auto &step : TX_POWER_STEPS) {
		if (step.level == level) {
			out[0] = static_cast<uint8_t>(step.dbm);
			return 1;
		}
	}
	return 0;
}

static int writeTxPower(const uint8_t *data, size_t len, uint8_t *, size_t) {
	if (len != 1) {
		return -1;
	}
	const int8_t dbm = static_cast<int8_t>(data[0]);
	for (const auto &step : TX_POWER_STEPS) {
		if (step.dbm == dbm) {
			return esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, step.level) == ESP_OK ? 0 : -1;
		}
	}
	return -1;
}

//======================= ADC config

static size_t readAdcConfig(uint8_t *out, size_t cap) {
	const AdcConfigNetworkData value = getAdcConfig();
	if (sizeof(value) > cap) {
		return 0;
	}
	memcpy(out, &value, sizeof(value));
	return sizeof(value);
}

//======================= KVS

// The frame is length-delimited end to end. The reply mirrors the request:
// [status][request]['='][command reply].
static int writeKvs(const uint8_t *data, size_t len, uint8_t *reply, size_t replyCap) {
	if (len + 2 >= replyCap) {
		return -1;
	}
	char *cmdReply = reinterpret_cast<char *>(reply) + len + 2;
	char status;
	switch (processKvsCommand(reinterpret_cast<const char *>(data), len, cmdReply,
	                          replyCap - len - 2)) {
	case KvsResult::Ok:
		status = KvsStatusOk;
		break;
	case KvsResult::Rejected:
		status = KvsStatusRejected;
		break;
	default:
		status = KvsStatusError;
		break;
	}
	reply[0] = status;
	memcpy(reply + 1, data, len);
	reply[len + 1] = '=';
	return static_cast<int>(len + 2 + strlen(cmdReply));
}

//======================= Characteristic instances

static Chr chrAdcFeed = {.notify = true};
static Chr chrKvs = {.write = writeKvs, .maxLen = USER_KVS_NETWORK_FRAME_LENGTH, .notify = true};
static Chr chrAdcConfig = {.read = readAdcConfig};
static Chr chrManufacturer = {.read = readManufacturer};
static Chr chrModel = {.read = readModel};
static Chr chrSerial = {.read = readSerial};
static Chr chrHwRevision = {.read = readHwRevision};
static Chr chrFirmware = {.read = readFirmware};
static Chr chrTxPowerRead = {.read = readTxPower};
static Chr chrTxPowerWrite = {.write = writeTxPower, .maxLen = 1};

static Chr *const ALL_CHRS[] = {
    &chrAdcFeed, &chrKvs,        &chrAdcConfig, &chrManufacturer, &chrModel,
    &chrSerial,  &chrHwRevision, &chrFirmware,  &chrTxPowerRead,  &chrTxPowerWrite,
};

static Chr *chrByHandle(uint16_t handle) {
	for (Chr *chr : ALL_CHRS) {
		if (chr->valHandle == handle) {
			return chr;
		}
	}
	return nullptr;
}

//======================= Dispatch

// One mbuf per call; ble_gatts_notify_custom frees the one it is handed.
static bool notifyChr(uint16_t connHandle, uint16_t valHandle, const uint8_t *data, size_t len) {
	os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
	if (!om) {
		return false;
	}
	return ble_gatts_notify_custom(connHandle, valHandle, om) == 0;
}

static int bleGattAccess(uint16_t connHandle, uint16_t, struct ble_gatt_access_ctxt *ctxt,
                         void *arg) {
	auto *chr = static_cast<Chr *>(arg);

	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR: {
		// Long reads (offset != 0) are refused: every value fits one packet.
		if (OS_MBUF_PKTLEN(ctxt->om) > 0) {
			return BLE_ATT_ERR_UNLIKELY;
		}
		const size_t len = chr->read(txBuf, sizeof(txBuf));
		if (len == 0) {
			return BLE_ATT_ERR_UNLIKELY;
		}
		return os_mbuf_append(ctxt->om, txBuf, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}

	case BLE_GATT_ACCESS_OP_WRITE_CHR: {
		uint16_t len = 0;
		if (ble_hs_mbuf_to_flat(ctxt->om, rxBuf, sizeof(rxBuf), &len) != 0) {
			return BLE_ATT_ERR_UNLIKELY;
		}
		if (len > chr->maxLen) {
			return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
		}
		const int replyLen = chr->write(rxBuf, len, txBuf, sizeof(txBuf));
		if (replyLen < 0) {
			return BLE_ATT_ERR_UNLIKELY;
		}
		if (replyLen > 0 && !notifyChr(connHandle, chr->valHandle, txBuf, replyLen)) {
			ESP_LOGW(TAG, "write reply notification lost");
		}
		return 0;
	}

	default:
		return BLE_ATT_ERR_UNLIKELY;
	}
}

//======================= GATT table

// Fully initializes the definition struct in one place (-Wmissing-field-
// initializers), so each table line is just UUID + state.
static ble_gatt_chr_def chrDef(const ble_uuid_t *uuid, Chr &chr) {
	return ble_gatt_chr_def{
	    .uuid = uuid,
	    .access_cb = bleGattAccess,
	    .arg = &chr,
	    .descriptors = nullptr,
	    .flags = chrFlags(chr),
	    .min_key_size = 0,
	    .val_handle = &chr.valHandle,
	    .cpfd = nullptr,
	};
}

static const struct ble_gatt_chr_def DEVICE_INFO_CHRS[] = {
    chrDef(&DEVICE_MAKE_NAME_CHR_UUID16.u, chrManufacturer),
    chrDef(&DEVICE_MODEL_NUMBER_CHR_UUID16.u, chrModel),
    chrDef(&DEVICE_SERIAL_NUMBER_CHR_UUID16.u, chrSerial),
    chrDef(&DEVICE_HARDWARE_REV_CHR_UUID16.u, chrHwRevision),
    chrDef(&DEVICE_FIRMWARE_VER_CHR_UUID16.u, chrFirmware),
    chrDef(&DEVICE_TX_POWER_CHR_UUID16.u, chrTxPowerRead),
    {},
};

static const struct ble_gatt_chr_def SAMPLER_CHRS[] = {
    chrDef(&ADC_FEED_CHR_UUID128.u, chrAdcFeed),
    chrDef(&USER_KVS_CHR_UUID128.u, chrKvs),
    chrDef(&ADC_CONF_CHR_UUID128.u, chrAdcConfig),
    {},
};

static const struct ble_gatt_chr_def TX_PWR_CHRS[] = {
    chrDef(&TX_PWR_CHR_UUID128.u, chrTxPowerWrite),
    {},
};

static const struct ble_gatt_svc_def SERVICES[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &DEVICE_INFO_SVC_UUID16.u,
     .includes = nullptr,
     .characteristics = DEVICE_INFO_CHRS},
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &DYNAMITE_SAMPLER_SVC_UUID128.u,
     .includes = nullptr,
     .characteristics = SAMPLER_CHRS},
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &TX_PWR_SVC_UUID128.u,
     .includes = nullptr,
     .characteristics = TX_PWR_CHRS},
    {},
};

//======================= Advertising and GAP

static BleServerEvents serverEvents{};
static uint16_t connHandle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t ownAddrType;

static int gapEvent(struct ble_gap_event *event, void *);

static void startAdvertising() {
	struct ble_hs_adv_fields scanRsp{};
	scanRsp.uuids128 = &DYNAMITE_SAMPLER_SVC_UUID128;
	scanRsp.num_uuids128 = 1;
	scanRsp.uuids128_is_complete = 1;
	const char *name = ble_svc_gap_device_name();
	scanRsp.name = reinterpret_cast<const uint8_t *>(name);
	scanRsp.name_len = strlen(name);
	scanRsp.name_is_complete = 1;
	if (const int rc = ble_gap_adv_rsp_set_fields(&scanRsp); rc != 0) {
		ESP_LOGE(TAG, "adv_rsp_set_fields: %d", rc);
	}

	struct ble_hs_adv_fields adv{};
	adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	if (const int rc = ble_gap_adv_set_fields(&adv); rc != 0) {
		ESP_LOGE(TAG, "adv_set_fields: %d", rc);
	}

	struct ble_gap_adv_params params{};
	params.conn_mode = BLE_GAP_CONN_MODE_UND;
	params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	if (const int rc =
	        ble_gap_adv_start(ownAddrType, nullptr, BLE_HS_FOREVER, &params, gapEvent, nullptr);
	    rc != 0) {
		ESP_LOGE(TAG, "adv_start: %d", rc);
	}
}

static int gapEvent(struct ble_gap_event *event, void *) {
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT: {
		if (event->connect.status != 0) {
			startAdvertising(); // the attempt failed; keep the door open
			break;
		}
		connHandle = event->connect.conn_handle;
		// tx_time is in 8 us units; (octets + 14) * 8 is what the controller
		// accepts for the maximum payload.
		if (ble_gap_set_data_len(connHandle, DLE_MAX_SIZE, (DLE_MAX_SIZE + 14) * 8) != 0) {
			ESP_LOGW(TAG, "set_data_len refused");
		}
		// Android accepts a 7.5 ms interval; iOS and Windows may keep their own.
		const struct ble_gap_upd_params params{
		    .itvl_min = 6,
		    .itvl_max = 24,
		    .latency = 0,
		    .supervision_timeout = 500,
		    .min_ce_len = 0,
		    .max_ce_len = 0,
		};
		if (ble_gap_update_params(connHandle, &params) != 0) {
			ESP_LOGW(TAG, "update_params refused");
		}
		if (serverEvents.onConnect) {
			serverEvents.onConnect(connHandle);
		}
		break;
	}

	case BLE_GAP_EVENT_DISCONNECT:
		if (serverEvents.onDisconnect) {
			serverEvents.onDisconnect(event->disconnect.reason);
		}
		connHandle = BLE_HS_CONN_HANDLE_NONE;
		startAdvertising();
		break;

	case BLE_GAP_EVENT_SUBSCRIBE:
		if (Chr *chr = chrByHandle(event->subscribe.attr_handle); chr && serverEvents.onSubscribe) {
			serverEvents.onSubscribe(chr->valHandle, event->subscribe.cur_notify);
		}
		break;

	case BLE_GAP_EVENT_DATA_LEN_CHG:
		if (serverEvents.onDataLengthChange) {
			serverEvents.onDataLengthChange(event->data_len_chg.max_tx_octets);
		}
		break;

	default:
		break;
	}
	return 0;
}

static void onSync() {
	if (const int rc = ble_hs_util_ensure_addr(0); rc != 0) {
		ESP_LOGE(TAG, "ensure_addr: %d", rc);
	}
	ble_hs_id_infer_auto(0, &ownAddrType);
	startAdvertising();
}

static void hostTask(void *) {
	nimble_port_run();
	nimble_port_freertos_deinit();
}

//======================= Entry points

bool bleHostStart(const char *deviceName, const BleServerEvents &events) {
	serverEvents = events;

	formatHwId();
	snprintf(deviceFirmware, sizeof(deviceFirmware), "%s|%s",
	         boardCfg() ? boardCfg()->name : BOARD_IDENTITY_NONE, GIT_DESCRIBE);

	if (const esp_err_t err = nimble_port_init(); err != ESP_OK) {
		ESP_LOGE(TAG, "nimble_port_init: %d", err);
		return false;
	}
	ble_hs_cfg.sync_cb = onSync;

	ble_svc_gap_init();
	ble_svc_gatt_init();
	if (ble_svc_gap_device_name_set(deviceName) != 0) {
		ESP_LOGE(TAG, "device name '%s' rejected", deviceName);
		return false;
	}

	// Pin the TX power so readTxPower always maps: until explicitly set, the
	// controller reports its implicit default of P3.
	if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3) != ESP_OK) {
		ESP_LOGE(TAG, "TX power pin refused");
		return false;
	}

	if (const int rc = ble_gatts_count_cfg(SERVICES); rc != 0) {
		ESP_LOGE(TAG, "count_cfg: %d", rc);
		return false;
	}
	if (const int rc = ble_gatts_add_svcs(SERVICES); rc != 0) {
		ESP_LOGE(TAG, "add_svcs: %d", rc);
		return false;
	}

	nimble_port_freertos_init(hostTask);
	return true;
}

uint16_t bleAdcFeedHandle(void) { return chrAdcFeed.valHandle; }

bool bleNotify(uint16_t valHandle, const void *data, size_t len) {
	if (connHandle == BLE_HS_CONN_HANDLE_NONE) {
		return false;
	}
	return notifyChr(connHandle, valHandle, static_cast<const uint8_t *>(data), len);
}
