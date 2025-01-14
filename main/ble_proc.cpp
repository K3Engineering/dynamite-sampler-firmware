#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include <NimBLEDevice.h>

#include "adc_ble_interface.h"
#include "ble_ota_interface.h"

#include "ble_proc.h"

#include "debug_pin.h"
#include <HardwareSerial.h>

//======================== <UUIDs>
constexpr char ADC_FEED_SVC_UUID[] = "e331016b-6618-4f8f-8997-1a2c7c9e5fa3";
constexpr char ADC_FEED_CHR_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

// static constexpr ble_uuid128_t ADC_FEED_SVC_UUID = BLE_UUID128_INIT(
//     0xa3, 0x5f, 0x9e, 0x7c, 0x2c, 0x1a, 0x97, 0x89, 0x8f, 0x4f, 0x18, 0x66, 0x6b, 0x01, 0x31,
//     0xe3);
// static constexpr ble_uuid128_t ADC_FEED_CHR_UUID = BLE_UUID128_INIT(
//     0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7, 0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5,
//     0xbe);
//======================== <\UUIDs>

static NimBLEServer         *bleServer                  = NULL;
static NimBLECharacteristic *blePublisherCharacteristic = NULL;
static uint16_t              adcNotifyChrHandle;

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
		Serial.print("On connect callback on core ");
		Serial.println(xPortGetCoreID());
	};

	void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
		NimBLEDevice::startAdvertising();
		Serial.print("On disco callback on core ");
		Serial.println(xPortGetCoreID());
	}
};

class AdcPublCallbacks : public NimBLECharacteristicCallbacks {
	void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
	                 uint16_t subValue) override {
		bleAccess.deviceConnected = subValue & 1;
		adcNotifyChrHandle        = connInfo.getConnHandle();
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
			blePublisherCharacteristic->notify(batch, bytesRead, adcNotifyChrHandle);
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
	Serial.println("Setting up BLE");
	// Create the BLE Device
	// Name the device with the mac address to make it unique for testing purposes.
	// TODO this probably isn't the elegant way to do this.
	char bleName[CONFIG_BT_NIMBLE_GAP_DEVICE_NAME_MAX_LEN];
	snprintf(bleName, sizeof(bleName), "DS %" PRIx64, ESP.getEfuseMac());
	NimBLEDevice::init(bleName);

	// Create the BLE Server
	bleServer = NimBLEDevice::createServer();
	static MyServerCallbacks serverCb;
	bleServer->setCallbacks(&serverCb, false);

	NimBLEService *srvAdcFeed = bleServer->createService(ADC_FEED_SVC_UUID);
	blePublisherCharacteristic =
	    srvAdcFeed->createCharacteristic(ADC_FEED_CHR_UUID, NIMBLE_PROPERTY::NOTIFY);
	static AdcPublCallbacks feedCb;
	blePublisherCharacteristic->setCallbacks(&feedCb);
	srvAdcFeed->start();

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
