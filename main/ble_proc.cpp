#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include <NimBLEDevice.h>
#include <NimBLELog.h>

#include "adc_ble_interface.h"
#include "ble_proc.h"
#include "debug_pin.h"
#include <HardwareSerial.h>

constexpr char SERVICE_UUID[]                 = "e331016b-6618-4f8f-8997-1a2c7c9e5fa3";
constexpr char ADC_FEED_CHARACTERISTIC_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

static NimBLEServer         *bleServer                     = NULL;
static NimBLECharacteristic *blePublisherCharacteristic    = NULL;
StreamBufferHandle_t         adcStreamBufferHandle         = NULL;
TaskHandle_t                 bleAdcFeedPublisherTaskHandle = NULL;

bool deviceConnected = false;

class MyServerCallbacks : public NimBLEServerCallbacks {
	void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
		deviceConnected = true;

		// TODO: Figure out:
		// is this needed?
		// Does this actually affect the packet size for the way we notify
		// Do we need to notify in a different way?
		server->setDataLen(connInfo.getConnHandle(), BLE_PUBL_DATA_DLE);
		Serial.print("On connect callback on core ");
		Serial.println(xPortGetCoreID());
	};

	void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
		deviceConnected = false;
		// TODO stop reading the ADC and stop the interupt when disconnected
		NimBLEDevice::startAdvertising();
		Serial.print("On disco callback on core ");
		Serial.println(xPortGetCoreID());
	}
};

// Read the adc buffer and update the BLE characteristic
static void blePublishAdcBuffer() {

	// TODO figure if should check deviceConnected?
	if (xStreamBufferBytesAvailable(adcStreamBufferHandle) >= ADC_FEED_CHUNK_SZ) {
		uint8_t batch[ADC_FEED_CHUNK_SZ];
		size_t  bytesRead = xStreamBufferReceive(adcStreamBufferHandle, batch, sizeof(batch), 0);
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

static void taskSetupBle(void *setupDone) {
	Serial.println("Setting up BLE");
	// Create the BLE Device
	// Name the device with the mac address to make it unique for testing purposes.
	// TODO this probably isn't the elegant way to do this.
	char bleName[35];
	sprintf(bleName, "DS %" PRIx64, ESP.getEfuseMac());
	NimBLEDevice::init(bleName);

	// Create the BLE Server
	bleServer = NimBLEDevice::createServer();
	bleServer->setCallbacks(new MyServerCallbacks());

	// Create the BLE Service
	NimBLEService *bleService = bleServer->createService(SERVICE_UUID);

	// Create a BLE Characteristic
	constexpr uint32_t prop = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
	                          NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE;
	blePublisherCharacteristic =
	    bleService->createCharacteristic(ADC_FEED_CHARACTERISTIC_UUID, prop);

	// Start the service
	bleService->start();

	// Start advertising
	NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	// Standard BLE advertisement packet is only 31 bytes, so long names don't always fit.
	// Scan response allows for devices to request more during the scan.
	// This will allow for more than the 31 bytes, like longer names.
	pAdvertising->setScanResponse(true);
	NimBLEDevice::startAdvertising();
}

void setupBle(int core) {
	// This buffer is to share the ADC values from the adc read task and BLE notify task
	adcStreamBufferHandle = xStreamBufferCreate(ADC_FEED_CHUNK_SZ * 8, 1);
	assert(adcStreamBufferHandle != NULL);

	volatile bool done = false;
	xTaskCreatePinnedToCore(taskSetupBle, "task_BLE_setup", 1024 * 5, (void *)&done, 1, NULL, core);
	while (!done)
		;
	delay(500);

	// TODO figure out priority for the BLE task
	xTaskCreatePinnedToCore(taksBlePublishAdcBuffer, "task_BLE_publish", 1024 * 5, NULL, 3,
	                        &bleAdcFeedPublisherTaskHandle, core);

	Serial.println("Waiting a client connection to notify...");
}
