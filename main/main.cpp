#include <Arduino.h>
#include <stdio.h>

#include "NimBLEDevice.h"
#include "NimBLELog.h"
#include "esp_mac.h"
#include "esp_pm.h"

// #include "ADS1120.h"
#include <freertos/stream_buffer.h>

#include "ADS131M0x.h"
#include "SPI.h"

#ifndef GIT_REVISION
#define GIT_REVISION "?"
#endif

// TODO since we don't need blue droid backwards compatability, all of the
// #defines could be expanded to their nimble equivalent
static NimBLEServer         *bleServer                  = NULL;
static NimBLECharacteristic *blePublisherCharacteristic = NULL;

static bool deviceConnected = false;

static StreamBufferHandle_t adcStreamBufferHandle         = NULL;
static TaskHandle_t         adcReadTaskHandle             = NULL;
static TaskHandle_t         bleAdcFeedPublisherTaskHandle = NULL;

static ADS131M0x adc;
static SPIClass  spiADC(HSPI);

constexpr char SERVICE_UUID[]                 = "e331016b-6618-4f8f-8997-1a2c7c9e5fa3";
constexpr char ADC_FEED_CHARACTERISTIC_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

// Core0 is for BLE
// Core1 is for everything else. Setup & loop run on core 1.
// ISR is handled on the core that sets it.
constexpr uint32_t CORE_BLE = 0;
constexpr uint32_t CORE_APP = 1;

// Nimble creates a GATT connection, which has some overhead.
constexpr uint16_t BLE_PUBL_DATA_DLE         = 251;
constexpr uint16_t BLE_PUBL_DATA_ATT_PAYLOAD = BLE_PUBL_DATA_DLE - 4 - 3;

#pragma pack(push, 1)
struct BleAdcFeedData {
	uint16_t status;
	uint8_t  data[sizeof(ADS131M0x::AdcRawOutput::data)];
	uint8_t  crc;
};
#pragma pack(pop)

constexpr size_t ADC_FEED_CHUNK_SZ =
    (BLE_PUBL_DATA_ATT_PAYLOAD / sizeof(BleAdcFeedData)) * sizeof(BleAdcFeedData);
static_assert(ADC_FEED_CHUNK_SZ <= BLE_PUBL_DATA_ATT_PAYLOAD);

// There are 2 pin on the v2.0.1 board that can be used for debugging.
constexpr uint8_t PIN_DEBUG_TOP = 47;
constexpr uint8_t PIN_DEBUG_BOT = 21;

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

static void setupBle() {
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
	Serial.println("Waiting a client connection to notify...");
}

// When we flag a piece of code with the IRAM_ATTR attribute, the compiled code
// is placed in the ESP32’s Internal RAM (IRAM). Otherwise the code is kept in
// Flash. And Flash on ESP32 is much slower than internal RAM.
void IRAM_ATTR isrAdcDrdy() {

	// unblock the task that will read the ADC & handle putting in the buffer
	if (adcReadTaskHandle != NULL) {
		BaseType_t taskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(adcReadTaskHandle, &taskWoken);
		portYIELD_FROM_ISR(taskWoken);
	}
}

// Read ADC values. If BLE device is connected, place them in the buffer.
// When accumulated enough, notify the ble task
static void adcReadAndBuffer() {

	ADS131M0x::AdcRawOutput adcReading = adc.rawReadADC();

	if (!deviceConnected)
		return;

	BleAdcFeedData toSend{.status = adcReading.status, .data = {}, .crc = adc.isCrcOk(&adcReading)};
	static_assert(sizeof(toSend.data) == sizeof(adcReading.data));
	memcpy(toSend.data, adcReading.data, sizeof(toSend.data));
	xStreamBufferSend(adcStreamBufferHandle, &toSend, sizeof(toSend), 0);
	// When the buffer is sufficiently large, time to send data.
	if (xStreamBufferBytesAvailable(adcStreamBufferHandle) >= ADC_FEED_CHUNK_SZ) {
		xTaskNotifyGive(bleAdcFeedPublisherTaskHandle);
	}
}

// Task that handles calling the read adc function and placing the values in the buffer.
static void taskAdcReadAndBuffer(void *) {

	while (true) {
		// Wait until ISR notifies this task. Normally numNotifications == 1,
		// numNotifications > 1 in case we cannot keep up with adc and have some data lost.
		uint32_t numNotifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (numNotifications > 0) [[likely]] {
			adcReadAndBuffer();
		}
	}
	vTaskDelete(NULL);
}

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

static void taskSetupAdc(void *setupDone) {
	Serial.print("setting up adc on core: ");
	Serial.println(xPortGetCoreID());
	constexpr uint8_t PIN_NUM_CLK   = 11;
	constexpr uint8_t PIN_NUM_MISO  = 10;
	constexpr uint8_t PIN_NUM_MOSI  = 9;
	constexpr uint8_t PIN_DRDY      = 12;
	constexpr uint8_t PIN_ADC_RESET = 14;
	constexpr uint8_t PIN_CS_ADC    = 13;

	pinMode(PIN_DEBUG_TOP, OUTPUT);
	pinMode(PIN_DEBUG_BOT, OUTPUT);

	adc.setClockSpeed(20000000); // SPI clock speed, has to run before adc.begin()
	adc.begin(&spiADC, PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI, PIN_CS_ADC, PIN_DRDY);

	// adc.setMultiplexer(0x00); // AIN0 AIN1
	// adc.setPGAbypass(0);

	adc.reset(PIN_ADC_RESET);

	adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);
	adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);
	adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);
	adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);

	adc.setChannelPGA(0, PGA_GAIN_1);
	adc.setChannelPGA(1, PGA_GAIN_32);
	adc.setChannelPGA(2, PGA_GAIN_32);

	adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);

	adc.setOsr(OSR_4096);

	uint16_t contents = adc.readRegister(0);
	Serial.print("register 0 contents ");
	Serial.println(contents);

	contents = adc.readRegister(1);
	Serial.print("register 1 contents ");
	Serial.println(contents);

	contents = adc.readRegister(2);
	Serial.print("register 2 contents ");
	Serial.println(contents);

	contents = adc.readRegister(3);
	Serial.print("register 3 contents ");
	Serial.println(contents);

	contents = adc.readRegister(4);
	Serial.print("register 4 contents ");
	Serial.println(contents);

	// TODO figure out if this function call is needed
	// The digitalPinToInterrupt() function takes a pin as an argument, and
	// returns the same pin if it can be used as an interrupt.
	//  Setup ISR to handle the falling edge of drdy
	attachInterrupt(digitalPinToInterrupt(PIN_DRDY), isrAdcDrdy, FALLING);

	// TODO figure out if you need to setup wake from sleep for gpio

	*(volatile bool *)setupDone = true;
	vTaskDelete(NULL);
}

extern "C" void app_main(void) {
	initArduino();

	Serial.begin(115200);
	Serial.println("Starting Arduino version:");
	Serial.println(ESP_ARDUINO_VERSION_STR);
	Serial.print("Git revision: ");
	Serial.println(GIT_REVISION);
	Serial.print("Running on Core: ");
	Serial.println(xPortGetCoreID());
	// Serial.print("Config PM SLP IRAM OPT (put lightsleep into ram):");
	// Serial.println(CONFIG_PM_SLP_IRAM_OPT);
	// TODO assert that this runs on core 1, so that all of the adc isr setup is
	// on core 1

	Serial.println("MAC address:");
	Serial.printf("0x%" PRIx64 "\n", ESP.getEfuseMac());

	// This buffer is to share the ADC values from the adc read task and BLE notify task
	adcStreamBufferHandle = xStreamBufferCreate(ADC_FEED_CHUNK_SZ * 8, 1);
	assert(adcStreamBufferHandle != NULL);

	// TODO figure out why BLE setup has to go first.
	setupBle();
	delay(500);

	volatile bool done = false;
	xTaskCreatePinnedToCore(taskSetupAdc, "task_ADC_setup", 1024 * 5, (void *)&done, 1, NULL,
	                        CORE_APP);
	while (!done)
		;
	delay(500);

	// TODO figure out the memory stack required
	const UBaseType_t priority = 30;
	xTaskCreatePinnedToCore(taskAdcReadAndBuffer, "task_ADC_read", 1024 * 5, NULL, priority,
	                        &adcReadTaskHandle, CORE_APP);

	// TODO figure out priority for the BLE task
	xTaskCreatePinnedToCore(taksBlePublishAdcBuffer, "task_BLE_publish", 1024 * 5, NULL, 3,
	                        &bleAdcFeedPublisherTaskHandle, CORE_BLE);

	// // esp_pm_config_esp32_t pm_config = {
	const esp_pm_config_t pmConfig = {
	    .max_freq_mhz       = 80,
	    .min_freq_mhz       = 10,
	    .light_sleep_enable = false,
	};
	esp_err_t err = esp_pm_configure(&pmConfig);
	Serial.print("pm err ");
	Serial.println(err);
	// ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
	Serial.println("Started!");
}
