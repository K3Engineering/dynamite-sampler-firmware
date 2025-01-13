#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include <NimBLEDevice.h>
#include <NimBLELog.h>

#include "adc_ble_interface.h"
#include "ble_proc.h"

#include "debug_pin.h"
#include <HardwareSerial.h>

constexpr char DEVICE_MANUFACTURER_NAME[] = "3K";
constexpr char DEVICE_MODEL_NUMBER[]      = "0.1d";

//======================== <UUIDs>
// constexpr char SERVICE_UUID[]                 = "e331016b-6618-4f8f-8997-1a2c7c9e5fa3";
// constexpr char ADC_FEED_CHARACTERISTIC_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

static constexpr ble_uuid128_t ADC_FEED_SVC_UUID = BLE_UUID128_INIT(
    0xa3, 0x5f, 0x9e, 0x7c, 0x2c, 0x1a, 0x97, 0x89, 0x8f, 0x4f, 0x18, 0x66, 0x6b, 0x01, 0x31, 0xe3);
static constexpr ble_uuid128_t ADC_FEED_CHR_UUID = BLE_UUID128_INIT(
    0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7, 0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

static constexpr ble_uuid16_t DEVICE_INFO_SVC_UUID =
    BLE_UUID16_INIT(0x180A); // BT Device Information Service
static constexpr ble_uuid16_t DEVICE_MAKE_NAME_CHR_UUID =
    BLE_UUID16_INIT(0x2A29); // BT Manufacturer Name String
static constexpr ble_uuid16_t DEVICE_MODEL_NUMBER_CHR_UUID =
    BLE_UUID16_INIT(0x2A24); // BT ModelNumberString

static constexpr ble_uuid128_t OTA_SVC_UUID = BLE_UUID128_INIT(
    0xd8, 0xe6, 0xfd, 0x1d, 0x4a, 024, 0xc6, 0xb1, 0x53, 0x4c, 0x4c, 0x59, 0x6d, 0xd9, 0xf1, 0xd6);
static constexpr ble_uuid128_t OTA_CONTROL_CHR_UUID = BLE_UUID128_INIT(
    0x30, 0xd8, 0xe3, 0x3a, 0x0e, 0x27, 0x22, 0xb7, 0xa4, 0x46, 0xc0, 0x21, 0xaa, 0x71, 0xd6, 0x7a);
static constexpr ble_uuid128_t OTA_DATA_CHR_UUID = BLE_UUID128_INIT(
    0xb0, 0xa5, 0xf8, 0x45, 0x8d, 0xca, 0x89, 0x9b, 0xd8, 0x4c, 0x40, 0x1f, 0x88, 0x88, 0x40, 0x23);
//======================== <\UUIDs>

static NimBLEServer         *bleServer                  = NULL;
static NimBLECharacteristic *blePublisherCharacteristic = NULL;

BleAccess bleAccess{
    .adcStreamBufferHandle         = NULL,
    .bleAdcFeedPublisherTaskHandle = NULL,
    .deviceConnected               = false,
};

class MyServerCallbacks : public NimBLEServerCallbacks {
	void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
		bleAccess.deviceConnected = true;

		// TODO: Figure out:
		// is this needed?
		// Does this actually affect the packet size for the way we notify
		// Do we need to notify in a different way?
		server->setDataLen(connInfo.getConnHandle(), BLE_PUBL_DATA_DLE);
		Serial.print("On connect callback on core ");
		Serial.println(xPortGetCoreID());
	};

	void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
		bleAccess.deviceConnected = false;
		// TODO stop reading the ADC and stop the interupt when disconnected
		NimBLEDevice::startAdvertising();
		Serial.print("On disco callback on core ");
		Serial.println(xPortGetCoreID());
	}
};

// Read the adc buffer and update the BLE characteristic
static void blePublishAdcBuffer() {

	// TODO figure if should check deviceConnected?
	if (xStreamBufferBytesAvailable(bleAccess.adcStreamBufferHandle) >= ADC_FEED_CHUNK_SZ) {
		uint8_t batch[ADC_FEED_CHUNK_SZ];
		size_t  bytesRead =
		    xStreamBufferReceive(bleAccess.adcStreamBufferHandle, batch, sizeof(batch), 0);
		if (bytesRead == ADC_FEED_CHUNK_SZ) {
			blePublisherCharacteristic->setValue(batch, bytesRead);
			blePublisherCharacteristic->notify();
		}
	}
}

// Task that is notified when the ADC buffer is ready to be sent
static void taksBlePublishAdcBuffer(void *) {

	while (true) {
		// This task is unblocked when the adc buffer is full and the characteristic
		// should be notified.
		uint32_t numNotifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (numNotifications > 0) [[likely]] {
			blePublishAdcBuffer();
		}
	}
	vTaskDelete(NULL);
}

#include "esp_ota_ops.h"

#define REBOOT_DEEP_SLEEP_TIMEOUT 500

typedef enum {
	SVR_CHR_OTA_CONTROL_NOP,
	SVR_CHR_OTA_CONTROL_REQUEST,
	SVR_CHR_OTA_CONTROL_REQUEST_ACK,
	SVR_CHR_OTA_CONTROL_REQUEST_NAK,
	SVR_CHR_OTA_CONTROL_DONE,
	SVR_CHR_OTA_CONTROL_DONE_ACK,
	SVR_CHR_OTA_CONTROL_DONE_NAK,
} svr_chr_ota_control_val_t;

typedef struct {
	uint16_t otaControlValHandle;
	uint16_t otaDataValHandle;

	const esp_partition_t *updatePartition;
	esp_ota_handle_t       updateHandle;
	uint16_t               numPkgsReceived;
	uint16_t               packetSize;

	uint8_t otaStatus;
	bool    updating;
} OtaControlData;

static OtaControlData otaControlData;

static void processOtaBegin(NimBLECharacteristic *pCharacteristic) {
	otaControlData.updatePartition = esp_ota_get_next_update_partition(NULL);
	esp_err_t err = esp_ota_begin(otaControlData.updatePartition, OTA_WITH_SEQUENTIAL_WRITES,
	                              &otaControlData.updateHandle);
	if (err == ESP_OK) {
		otaControlData.otaStatus = SVR_CHR_OTA_CONTROL_REQUEST_ACK;
		otaControlData.updating  = true;
	} else {
		esp_ota_abort(otaControlData.updateHandle);
		otaControlData.otaStatus    = SVR_CHR_OTA_CONTROL_REQUEST_NAK;
		otaControlData.updateHandle = 0;
		otaControlData.updating     = false;
		// ESP_LOGE(TAG, "esp_ota_begin error %d (%s)", err, esp_err_to_name(err));
		Serial.println("esp_ota_begin error");
	}
	otaControlData.numPkgsReceived = 0;

	// notify the client via BLE that the OTA has been acknowledged (or not)
	pCharacteristic->setValue(otaControlData.otaStatus);
	pCharacteristic->notify();
	// ESP_LOGI(TAG, "OTA request (n)ack %u", otaControlData.otaStatus);
	Serial.print("OTA request (n)ack ");
	Serial.println(otaControlData.otaStatus);

	return;
}

static void processOtaDone(NimBLECharacteristic *pCharacteristic) {
	Serial.println("processOtaDone");
	otaControlData.updating = false;
	Serial.println(__LINE__);
	esp_err_t err = esp_ota_end(otaControlData.updateHandle);
	Serial.println(__LINE__);
	if (err == ESP_OK) {
		err = esp_ota_set_boot_partition(otaControlData.updatePartition);
		// ESP_LOGE(TAG, "esp_ota_set_boot_partition=%d (%s)!", err,
		//      esp_err_to_name(err))
		Serial.println((err == ESP_OK) ? "set_boot_partition ok" : "set_boot_partition failed");
	} else {
		// ESP_LOGE(TAG, "esp_ota_end failed %d (%s)!", err, esp_err_to_name(err));
		Serial.println("esp_ota_end failed");
	}

	otaControlData.otaStatus =
	    (err == ESP_OK) ? SVR_CHR_OTA_CONTROL_DONE_ACK : SVR_CHR_OTA_CONTROL_DONE_NAK;
	// notify the client via BLE that DONE has been acknowledged
	pCharacteristic->setValue(otaControlData.otaStatus);

	// restart the ESP to finish the OTA
	if (err == ESP_OK) {
		Serial.println("Preparing to restart!");
		vTaskDelay(pdMS_TO_TICKS(REBOOT_DEEP_SLEEP_TIMEOUT));
		esp_restart();
	} else {
		Serial.println("OTA FAILED");
	}
}

class OtaControlChrCallbacks : public NimBLECharacteristicCallbacks {
	void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		Serial.println("onRead");
		pCharacteristic->setValue(otaControlData.otaStatus);
	}
	void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		const uint16_t omLen = pCharacteristic->getLength();
		if (sizeof(uint8_t) == omLen) {
			NimBLEAttValue val  = pCharacteristic->getValue();
			uint8_t        code = *val.begin();
			if (SVR_CHR_OTA_CONTROL_REQUEST == code) {
				processOtaBegin(pCharacteristic);
			} else if (SVR_CHR_OTA_CONTROL_DONE == code) {
				processOtaDone(pCharacteristic);
			}
		}
	}
	void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
	                 uint16_t subValue) override {
		Serial.println("onSubscribe");
	}
};

static constexpr uint16_t decodeOtaPacketSz(const uint8_t *netData) {
	return netData[0] + (netData[1] << 8);
}

class OtaDataChrCallbacks : public NimBLECharacteristicCallbacks {
	void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		Serial.print("onWrite Data omLen ");
		const uint16_t omLen = pCharacteristic->getLength();
		Serial.println(omLen);
		if (otaControlData.updating) {
			NimBLEAttValue val = pCharacteristic->getValue();
			// write the data chunk to the partition
			esp_err_t err = esp_ota_write(otaControlData.updateHandle, val.begin(), val.size());
			if (err != ESP_OK) {
				Serial.print("esp_ota_write failed err: ");
				Serial.println(err);
			}
			otaControlData.numPkgsReceived++;
			Serial.print("Received packet ");
			Serial.println(otaControlData.numPkgsReceived);
			Serial.println(val.size());
		} else {
			if (sizeof(otaControlData.packetSize) == omLen) {
				NimBLEAttValue val        = pCharacteristic->getValue();
				otaControlData.packetSize = decodeOtaPacketSz(val.begin());
				Serial.print("Packet size: ");
				Serial.println(otaControlData.packetSize);
			}
		}
	}
};

void checkOtaStatus(void) {
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t   ota_state;
	if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
			bool diagnostic_is_ok = true; // TODO: run diagnostic function
			if (diagnostic_is_ok) {
				Serial.println("Diagnostics completed successfully! Continuing execution ...");
				esp_ota_mark_app_valid_cancel_rollback();
			} else {
				Serial.println("Diagnostics failed! Start rollback to the previous version ...");
				esp_ota_mark_app_invalid_rollback_and_reboot();
			}
		}
	}
}

static void taskSetupBle(void *setupDone) {
	Serial.println("Setting up BLE");
	// Create the BLE Device
	// Name the device with the mac address to make it unique for testing purposes.
	// TODO this probably isn't the elegant way to do this.
	char bleName[CONFIG_BT_NIMBLE_GAP_DEVICE_NAME_MAX_LEN];
	snprintf(bleName, sizeof(bleName), "DS %" PRIx64, ESP.getEfuseMac());
	NimBLEDevice::init(bleName);

	// Create the BLE Server
	bleServer = NimBLEDevice::createServer();
	static MyServerCallbacks callbacs;
	bleServer->setCallbacks(&callbacs, false);

	// Create the BLE Services
	NimBLEService        *srvDeviceInfo = bleServer->createService(DEVICE_INFO_SVC_UUID.value);
	NimBLECharacteristic *chrDevName    = srvDeviceInfo->createCharacteristic(
        DEVICE_MAKE_NAME_CHR_UUID.value, NIMBLE_PROPERTY::READ, sizeof(DEVICE_MANUFACTURER_NAME));
	chrDevName->setValue(DEVICE_MANUFACTURER_NAME);
	NimBLECharacteristic *chrDevModelNum = srvDeviceInfo->createCharacteristic(
	    DEVICE_MODEL_NUMBER_CHR_UUID.value, NIMBLE_PROPERTY::READ, sizeof(DEVICE_MODEL_NUMBER));
	chrDevModelNum->setValue(DEVICE_MODEL_NUMBER);

	NimBLEService *srvAdcFeed = bleServer->createService(&ADC_FEED_SVC_UUID);
	blePublisherCharacteristic =
	    srvAdcFeed->createCharacteristic(&ADC_FEED_CHR_UUID, NIMBLE_PROPERTY::NOTIFY);

	NimBLEService        *srvOTA        = bleServer->createService(&OTA_SVC_UUID);
	NimBLECharacteristic *chrOtaControl = srvOTA->createCharacteristic(
	    &OTA_CONTROL_CHR_UUID,
	    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
	static OtaControlChrCallbacks otaControlCb;
	chrOtaControl->setCallbacks(&otaControlCb);

	NimBLECharacteristic *chrOtaData =
	    srvOTA->createCharacteristic(&OTA_DATA_CHR_UUID, NIMBLE_PROPERTY::WRITE);
	static OtaDataChrCallbacks otaDataCb;
	chrOtaData->setCallbacks(&otaDataCb);

	// Start the service
	srvAdcFeed->start();
	srvDeviceInfo->start();
	srvOTA->start();

	// Start advertising
	NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
	pAdvertising->setName("3K Probe");
	pAdvertising->addServiceUUID(srvAdcFeed->getUUID());
	// pAdvertising->addServiceUUID(srvDeviceInfo->getUUID());
	// pAdvertising->addServiceUUID(srvOTA->getUUID());
	//  Standard BLE advertisement packet is only 31 bytes, so long names don't always fit.
	//  Scan response allows for devices to request more during the scan.
	//  This will allow for more than the 31 bytes, like longer names.
	//  pAdvertising->enableScanResponse(true);
	NimBLEDevice::startAdvertising();

	*(volatile bool *)setupDone = true;
	vTaskDelete(NULL);
}

void setupBle(int core) {
	// This buffer is to share the ADC values from the adc read task and BLE notify task
	bleAccess.adcStreamBufferHandle = xStreamBufferCreate(ADC_FEED_CHUNK_SZ * 8, 1);
	assert(bleAccess.adcStreamBufferHandle != NULL);

	volatile bool done = false;
	xTaskCreatePinnedToCore(taskSetupBle, "task_BLE_setup", 1024 * 5, (void *)&done, 1, NULL, core);
	while (!done)
		vTaskDelay(10);

	// TODO figure out priority for the BLE task
	xTaskCreatePinnedToCore(taksBlePublishAdcBuffer, "task_BLE_publish", 1024 * 5, NULL, 3,
	                        &bleAccess.bleAdcFeedPublisherTaskHandle, core);

	Serial.println("Waiting a client connection to notify...");
}
