#include <esp_log.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include <NimBLEDevice.h>

#include "adc_ble_interface.h"
#include "adc_proc.h"
#include "ble_ota_interface.h"
#include "ble_proc.h"
#include "dynamite_uuid.h"
#include "loadcell_calibration.h"

#include "build_metadata.h"

constexpr char TAG[] = "BLE";

static NimBLEServer *bleServer = NULL;

static NimBLECharacteristic *chrAdcFeed = NULL;

static uint16_t adcFeedConnectionHandle = 0; // TODO: rename

BleAccess bleAccess{
    .adcStreamBufferHandle         = NULL,
    .bleAdcFeedPublisherTaskHandle = NULL,
    .clientSubscribed              = false,
};

class MyServerCallbacks : public NimBLEServerCallbacks {
	void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
		// TODO: Figure out:
		// is this needed?
		// Does this actually affect the packet size for the way we notify
		// Do we need to notify in a different way?
		server->setDataLen(connInfo.getConnHandle(), BLE_PUBL_DATA_DLE);

		// Best case, devices can handle 7.5ms interval connection (android phone)
		// older iOS might be 15ms, Windows PC might be 30ms. Connection interval is set in
		// increments of 1.25ms.
		// Don't skip any connection intervals, and timout after 10*N ms
		server->updateConnParams(connInfo.getConnHandle(), 6, 6 * 4, 0, 500);
		ESP_LOGD(TAG, "Server onConnect, core %u", xPortGetCoreID());
	};

	void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
		NimBLEDevice::startAdvertising();
		ESP_LOGD(TAG, "Server onDisco, core %u", xPortGetCoreID());
	}
};

class TxPowerPublisherCallbacks : public NimBLECharacteristicCallbacks {
	void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		// Read by a client.
		int power = NimBLEDevice::getPower();

		TxPowerNetworkData rPwr{
		    .val = static_cast<int8_t>(power),
		};
		pCharacteristic->setValue(rPwr);
		ESP_LOGD(TAG, "Updated TX power chr with value: x%x (getPower=x%x)", rPwr.val, power);
	}
};

// Set the BLE standardized device info
static void setupDeviceInfo(NimBLEServer *server) {
	NimBLEService *srvDeviceInfo = server->createService(DEVICE_INFO_SVC_UUID16.value);
	{ // Manufacturer
		NimBLECharacteristic *chr = srvDeviceInfo->createCharacteristic(
		    DEVICE_MAKE_NAME_CHR_UUID16.value, NIMBLE_PROPERTY::READ,
		    sizeof(DEVICE_MANUFACTURER_NAME));
		chr->setValue(DEVICE_MANUFACTURER_NAME);
		ESP_LOGI(TAG, "Set Device manufacturer name to: %s", DEVICE_MANUFACTURER_NAME);
	}
	{ // Firmware version
		NimBLECharacteristic *chr = srvDeviceInfo->createCharacteristic(
		    DEVICE_FIRMWARE_VER_CHR_UUID16.value, NIMBLE_PROPERTY::READ, sizeof(GIT_DESCRIBE));
		chr->setValue(GIT_DESCRIBE);
		ESP_LOGI(TAG, "Set Device Firmware version to: %s", GIT_DESCRIBE);
	}
	{ // Transmitter power
		NimBLECharacteristic *chr = srvDeviceInfo->createCharacteristic(
		    DEVICE_TX_POWER_CHR_UUID16.value, NIMBLE_PROPERTY::READ, sizeof(TxPowerNetworkData));
		static TxPowerPublisherCallbacks cb;
		chr->setCallbacks(&cb);
	}
	srvDeviceInfo->start();
}

class TxPowerManagerCallbacks : public NimBLECharacteristicCallbacks {
	void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		// Value written to the characteristic by a client.
		const NimBLEAttValue val = pCharacteristic->getValue();
		if (val.length() != sizeof(TxPowerNetworkData)) {
			ESP_LOGW(TAG, "TX power onWrite, recieved value of length %u", val.length());
			return;
		}
		TxPowerNetworkData power = *(TxPowerNetworkData *)val.data();
		if (!NimBLEDevice::setPower(power.val)) {
			ESP_LOGE(TAG, "Set TX power failed");
		}
	}
};

static void setupPowerManagerInterface(NimBLEServer *server) {
	NimBLEService *srvc = bleServer->createService(&TX_PWR_SVC_UUID128);
	{ // Set transmitter power - write
		NimBLECharacteristic *chr = srvc->createCharacteristic(
		    &TX_PWR_CHR_UUID128, NIMBLE_PROPERTY::WRITE, sizeof(TxPowerNetworkData));
		static TxPowerManagerCallbacks cb;
		chr->setCallbacks(&cb);
	}
	srvc->start();
}

class AdcFeedCallbacks : public NimBLECharacteristicCallbacks {
	void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
	                 uint16_t subValue) override {
		bleAccess.clientSubscribed = subValue & 1;
		if (bleAccess.clientSubscribed) {
			adcFeedConnectionHandle = connInfo.getConnHandle();
			startAdc();
		} else {
			stopAdc();
			adcFeedConnectionHandle = 0;
		}
	}
};

class AdcConfigCallbacks : public NimBLECharacteristicCallbacks {
	void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		pCharacteristic->setValue(getAdcConfig());
	}
};

static void setupAdcFeed(NimBLEServer *server) {
	NimBLEService *srvc = bleServer->createService(&DYNAMITE_SAMPLER_SVC_UUID128);
	{ // ADC feed
		chrAdcFeed = srvc->createCharacteristic(&ADC_FEED_CHR_UUID128, NIMBLE_PROPERTY::NOTIFY);
		static AdcFeedCallbacks cb;
		chrAdcFeed->setCallbacks(&cb);
	}
	{ // Calibration data
		NimBLECharacteristic *chr =
		    srvc->createCharacteristic(&LC_CALIB_CHR_UUID128, NIMBLE_PROPERTY::READ);
		CalibrationNetworkData calibrData;
		if (readLoadcellCalibration(&calibrData)) {
			chr->setValue(calibrData);
		}
	}
	{ // ADC config
		NimBLECharacteristic *chr =
		    srvc->createCharacteristic(&ADC_CONF_CHR_UUID128, NIMBLE_PROPERTY::READ);
		static AdcConfigCallbacks cb;
		chr->setCallbacks(&cb);
	}
	srvc->start();
}

static void setupAdvertising(const char *name) {
	// Start advertising
	NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
	//  Standard BLE advertisement packet is only 31 bytes, so long names don't always fit.
	//  Scan response allows for devices to request more during the scan.
	//  This will allow for more than the 31 bytes, like longer names.
	// If your device is battery powered you may consider setting scan response *to false as
	// it will extend battery life at the expense of less data sent.
	pAdvertising->enableScanResponse(true);
	pAdvertising->setName(name);
	pAdvertising->addServiceUUID(&DYNAMITE_SAMPLER_SVC_UUID128);
	// pAdvertising->addServiceUUID(srvDeviceInfo->getUUID());
	// pAdvertising->addServiceUUID(srvOTA->getUUID());
	NimBLEDevice::startAdvertising();
}

// Read the adc buffer and update the BLE characteristic
static void blePublishAdcBuffer() {
	AdcFeedNetworkPacket packet;
	static_assert(sizeof(packet) <= BLE_PUBL_DATA_ATT_PAYLOAD);
	if (xStreamBufferBytesAvailable(bleAccess.adcStreamBufferHandle) >= sizeof(packet.adc)) {
		size_t bytesRead = xStreamBufferReceive(bleAccess.adcStreamBufferHandle, &packet.adc,
		                                        sizeof(packet.adc), 0);
		if (bytesRead == sizeof(packet.adc)) {
			static uint16_t count             = 0;
			packet.hdr.sample_sequence_number = htole16(count);
			chrAdcFeed->notify(packet, adcFeedConnectionHandle);
			count += sizeof(packet.adc) / sizeof(*packet.adc);
		}
	}
}

// Task that is notified when the ADC buffer is ready to be sent
static void taskBlePublishAdcBuffer(void *) {
	while (true) {
		// This task is unblocked every ADC tick, provided
		// the buffer has at least ADC_FEED_CHUNK_SZ bytes
		// so this end receives notifications more frequently when
		// there is extra data in the queue
		uint32_t numNotifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (numNotifications > 0) [[likely]] {
			blePublishAdcBuffer();
		}
	}
	vTaskDelete(NULL);
}

static void taskSetupBle(void *setupDone) {
	ESP_LOGI(TAG, "Setting up BLE");
	// Create the BLE Device
	// Name the device with the mac address to make it unique for testing purposes.
	// TODO this probably isn't the elegant way to do this.
	uint8_t mac[8]; // size - see esp_efuse_mac_get_default() docs.
	esp_efuse_mac_get_default(mac);
	char bleName[CONFIG_BT_NIMBLE_GAP_DEVICE_NAME_MAX_LEN];
	snprintf(bleName, sizeof(bleName), "DS %02x%02x%02x%02x%02x%02x", mac[5], mac[4], mac[3],
	         mac[2], mac[1], mac[0]);
	NimBLEDevice::init(bleName);

	NimBLEDevice::setMTU(BLE_ATT_MTU_MAX);

	// Create the BLE Server
	bleServer = NimBLEDevice::createServer();
	static MyServerCallbacks cb;
	bleServer->setCallbacks(&cb, false);

	setupAdcFeed(bleServer);
	setupDeviceInfo(bleServer);
	setupPowerManagerInterface(bleServer);
	setupBleOta(bleServer);

	setupAdvertising(bleName);
	ESP_LOGI(TAG, "BLE setup done, advertising started");

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
	xTaskCreatePinnedToCore(taskBlePublishAdcBuffer, "task_BLE_publish", 1024 * 5, NULL, 3,
	                        &bleAccess.bleAdcFeedPublisherTaskHandle, core);

	ESP_LOGI(TAG, "Waiting a client connection to notify...");
}
