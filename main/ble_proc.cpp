#include <esp_log.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include <NimBLEDevice.h>

#include "adc_ble_interface.h"
#include "ble_ota_interface.h"
#include "ble_proc.h"
#include "dynamite_uuid.h"
#include "loadcell_calibration.h"

constexpr char TAG[] = "BLE";

static NimBLEServer         *bleServer                = NULL;
static NimBLECharacteristic *adcFeedBleCharacteristic = NULL;
static uint16_t              adcFeedConnectionHandle; // TODO: rename

BleAccess bleAccess{
    .adcStreamBufferHandle         = NULL,
    .bleAdcFeedPublisherTaskHandle = NULL,
    .deviceConnected               = false,
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

class AdcFeedCallbacks : public NimBLECharacteristicCallbacks {
	void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
	                 uint16_t subValue) override {
		bleAccess.deviceConnected = subValue & 1;
		adcFeedConnectionHandle   = connInfo.getConnHandle();
		//  TODO: stop reading the ADC and stop the interupt when disconnected
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
			adcFeedBleCharacteristic->notify(batch, bytesRead, adcFeedConnectionHandle);
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
	static MyServerCallbacks serverCb;
	bleServer->setCallbacks(&serverCb, false);

	NimBLEService *srvAdcFeed = bleServer->createService(&DYNAMITE_SAMPLER_SVC_UUID128);
	adcFeedBleCharacteristic =
	    srvAdcFeed->createCharacteristic(&ADC_FEED_CHR_UUID128, NIMBLE_PROPERTY::NOTIFY);
	static AdcFeedCallbacks feedCb;
	adcFeedBleCharacteristic->setCallbacks(&feedCb);

	NimBLECharacteristic *calibrationCharacteristic =
	    srvAdcFeed->createCharacteristic(&LC_CALIB_CHR_UUID128, NIMBLE_PROPERTY::READ);
	CalibrationData calibration;
	if (readLoadcellCalibration(&calibration)) {
		calibrationCharacteristic->setValue(calibration.data, sizeof(calibration.data));
	}

	srvAdcFeed->start();

	setDeviceInfo(bleServer);
	setupBleOta(bleServer);

	// Start advertising
	NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
	pAdvertising->setName(bleName);
	pAdvertising->addServiceUUID(srvAdcFeed->getUUID());
	// pAdvertising->addServiceUUID(srvDeviceInfo->getUUID());
	// pAdvertising->addServiceUUID(srvOTA->getUUID());

	//  Standard BLE advertisement packet is only 31 bytes, so long names don't always fit.
	//  Scan response allows for devices to request more during the scan.
	//  This will allow for more than the 31 bytes, like longer names.
	// If your device is battery powered you may consider setting scan response *to false as it
	// will extend battery life at the expense of less data sent.
	pAdvertising->enableScanResponse(true);

	NimBLEDevice::startAdvertising();
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
	xTaskCreatePinnedToCore(taksBlePublishAdcBuffer, "task_BLE_publish", 1024 * 5, NULL, 3,
	                        &bleAccess.bleAdcFeedPublisherTaskHandle, core);

	ESP_LOGI(TAG, "Waiting a client connection to notify...");
}
