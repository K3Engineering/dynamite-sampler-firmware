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

extern "C" {
void app_main(void);
}

// TODO since we don't need blue droid backwards compatability, all of the
// #defines could be expanded to their nimble equivalent
BLEServer         *pServer         = NULL;
BLECharacteristic *pCharacteristic = NULL;

bool deviceConnected = false;
// bool oldDeviceConnected = false;

StreamBufferHandle_t xStreamBuffer    = NULL;
TaskHandle_t         xHandleADCRead   = NULL; // Used by ISR to unblock ADC read and proccesing
TaskHandle_t         xHandleNotifyBLE = NULL;

#define SERVICE_UUID        "e331016b-6618-4f8f-8997-1a2c7c9e5fa3"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

ADS131M0x adc;
SPIClass  SpiADC(HSPI);

// Core0 is for BLE
// Core1 is for everything else. Setup & loop run on core 1.
// ISR is handled on the core that sets it.
const int CORE_BLE = 0;
const int CORE_APP = 1;

// There are 2 pin on the v2.0.1 board that can be used for debugging.
const int PIN_DEBUG_TOP = 47;
const int PIN_DEBUG_BOT = 21;
// For debugging ISR and adc handle function
const int PIN_DEBUG_ISR      = PIN_DEBUG_TOP;
const int PIN_DEBUG_ADC_TASK = PIN_DEBUG_BOT;

class MyServerCallbacks : public BLEServerCallbacks {
	// void onConnect(BLEServer *pServer) {
	void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) {
		deviceConnected = true;

		// TODO: Figure out:
		// is this needed?
		// Does this actually affect the packet size for the way we notify
		// Do we need to notify in a different way?
		pServer->setDataLen(connInfo.getConnHandle(), 251);
		Serial.print("On connect callback on core ");
		Serial.println(xPortGetCoreID());
	};

	// void onDisconnect(BLEServer *pServer) {
	void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) {
		deviceConnected = false;
		// TODO stop reading the ADC and stop the interupt when disconnected

		BLEDevice::startAdvertising();
		Serial.print("On disco callback on core ");
		Serial.println(xPortGetCoreID());
	}
};

void setup_ble() {
	Serial.println("Setting up BLE");
	// Create the BLE Device
	// Name the device with the mac address to make it unique for testing purposes.
	// TODO this probably isn't the elegant way to do this.
	char ble_name[35] = {0};
	sprintf(ble_name, "DS %" PRIx64 "\n", ESP.getEfuseMac());
	const std::string ble_name_str = std::string(ble_name);
	BLEDevice::init(ble_name_str);

	// Create the BLE Server
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// Create the BLE Service
	BLEService *pService = pServer->createService(SERVICE_UUID);

	// Create a BLE Characteristic
	uint32_t prop = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY |
	                NIMBLE_PROPERTY::INDICATE;
	pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, prop);

	// Create a BLE Descriptor
	// Automatically created by nimble. Needed for bluedroid
	// pCharacteristic->addDescriptor(new BLE2902());

	// Start the service
	pService->start();

	// Start advertising
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	// Standard BLE advertisement packet is only 31 bytes, so long names don't always fit.
	// Scan response allows for devices to request more during the scan.
	// This will allow for more than the 31 bytes, like longer names.
	pAdvertising->setScanResponse(true);
	BLEDevice::startAdvertising();
	Serial.println("Waiting a client connection to notify...");
}

// When we flag a piece of code with the IRAM_ATTR attribute, the compiled code
// is placed in the ESP32’s Internal RAM (IRAM). Otherwise the code is kept in
// Flash. And Flash on ESP32 is much slower than internal RAM.
void IRAM_ATTR isr_adc_drdy() {

	// unblock the task that will read the ADC & handle putting in the buffer
	if (xHandleADCRead != NULL) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(xHandleADCRead, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

// Read ADC values. If BLE device is connected, place them in the buffer.
// If the buffer is large enough, write the buffer to the BLE characteristic.
void adc_read_and_buffer() {
	digitalWrite(PIN_DEBUG_ISR, HIGH);

	adcOutput adcReading = adc.readADC();
	uint32_t  adcValue   = adcReading.ch2;
	// Only place ADC reading if the the BLE device is connected.
	// TODO - this probably should be handled in a different way.
	if (deviceConnected) {
		xStreamBufferSend(xStreamBuffer, &adcValue, 3, 0); // 3 bytes instead of sizeof(adcValue)

		// When the buffer is sufficiently large, time to send data.
		// BLE allows to extend data packet from 27 to 251 bytes
		// Schedule a task when buffer is closer to the upper limit
		// TODO figure out the best number or a way to know when to schedule
		if (xStreamBufferBytesAvailable(xStreamBuffer) > 200) {
			xTaskNotifyGive(xHandleNotifyBLE);
		}
	}
	digitalWrite(PIN_DEBUG_ISR, LOW);
}

// Task that handles calling the read adc function and placing the values in the buffer.
void task_adc_read_and_buffer(void *parameter) {
	// Durtation until the wait for ISR times out
	const TickType_t xBlockTime = pdMS_TO_TICKS(500);
	while (true) {
		// Wait until ISR notifies this task, or time out. ISR can notify multiple times,
		// but this will reset the counter to zero each time.
		// How many times did the ISR notify this task
		uint32_t ulNotifiedValue = ulTaskNotifyTake(pdTRUE, xBlockTime);
		if (ulNotifiedValue > 0) {
			adc_read_and_buffer();
		} else {
			// time out happened, TBD how this should be handled
		}
	}
	vTaskDelete(NULL);
}

// Read the adc buffer and update the BLE characteristic
void taks_notify_ble(void *parameter) {
	const TickType_t xBlockTime = pdMS_TO_TICKS(500);
	while (true) {
		// This task is unblocked when the adc buffer is full and the characteristic
		// should be notified.
		uint32_t ulNotifiedValue = ulTaskNotifyTake(pdTRUE, xBlockTime);
		if (ulNotifiedValue > 0) {
			digitalWrite(PIN_DEBUG_ADC_TASK, HIGH);
			// TODO verify that 251 is the actual max that can be sent and not
			// 251 minus some header.
			// TODO figure if should check deviceConnected?
			uint8_t batch[251];
			size_t  bytesRead = 0;

			bytesRead = xStreamBufferReceive(xStreamBuffer, batch, 251, 0);

			if (bytesRead > 0) {

				pCharacteristic->setValue(batch, bytesRead);
				pCharacteristic->notify();
			}
			digitalWrite(PIN_DEBUG_ADC_TASK, LOW);
		} else {
			// time out happened, TBD how this should be handled
		}
	}
}

void task_setup_adc(void *parameter) {
	Serial.print("setting up adc on core: ");
	Serial.println(xPortGetCoreID());
	uint8_t PIN_NUM_CLK   = 11;
	uint8_t PIN_NUM_MISO  = 10;
	uint8_t PIN_NUM_MOSI  = 9;
	uint8_t PIN_DRDY      = 12;
	uint8_t PIN_ADC_RESET = 14;
	uint8_t PIN_CS_ADC    = 13;

	pinMode(PIN_DEBUG_TOP, OUTPUT);
	pinMode(PIN_DEBUG_BOT, OUTPUT);

	adc.setClockSpeed(20000000); // SPI clock speed, has to run before adc.begin()
	adc.begin(&SpiADC, PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI, PIN_CS_ADC, PIN_DRDY);

	// adc.setMultiplexer(0x00); // AIN0 AIN1
	// adc.setPGAbypass(0);

	adc.reset(PIN_ADC_RESET);

	adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);
	adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);
	adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);
	adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);

	adc.setChannelPGA(0, PGA_GAIN_1);
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

	xStreamBuffer = xStreamBufferCreate(2000, 1);
	assert(xStreamBuffer != NULL);

	// TODO figure out if this function call is needed
	// The digitalPinToInterrupt() function takes a pin as an argument, and
	// returns the same pin if it can be used as an interrupt.
	//  Setup ISR to handle the falling edge of drdy
	attachInterrupt(digitalPinToInterrupt(PIN_DRDY), isr_adc_drdy, FALLING);

	// TODO figure out if you need to setup wake from sleep for gpio

	vTaskDelete(NULL);
}

void app_main(void) {
	initArduino();
	Serial.begin(115200);
	Serial.println("Starting Arduino version:");
	Serial.println(ESP_ARDUINO_VERSION_STR);
	Serial.print("Running on Core: ");
	Serial.println(xPortGetCoreID());
	// Serial.print("Config PM SLP IRAM OPT (put lightsleep into ram):");
	// Serial.println(CONFIG_PM_SLP_IRAM_OPT);
	// TODO assert that this runs on core 1, so that all of the adc isr setup is
	// on core 1

	Serial.println("MAC address:");
	Serial.printf("0x%" PRIx64 "\n", ESP.getEfuseMac());

	// TODO figure out why BLE setup has to go first.
	setup_ble();
	delay(500);

	xTaskCreatePinnedToCore(task_setup_adc, "task_ADC_setup", 5000, NULL, 1, NULL, CORE_APP);
	delay(500);

	// TODO figure out the memory stack required
	UBaseType_t priority = 30;
	xTaskCreatePinnedToCore(task_adc_read_and_buffer, "task_ADC_read_buffer", 5000, NULL, priority,
	                        &xHandleADCRead, CORE_APP);

	// TODO figure out priority for the BLE task
	xTaskCreatePinnedToCore(taks_notify_ble, "task_notify_BLE", 5000, NULL, 3, &xHandleNotifyBLE,
	                        CORE_BLE);

	// // esp_pm_config_esp32_t pm_config = {
	esp_pm_config_t pm_config = {
	    .max_freq_mhz       = 80,
	    .min_freq_mhz       = 10,
	    .light_sleep_enable = false,
	};
	esp_err_t err = esp_pm_configure(&pm_config);
	Serial.print("pm err ");
	Serial.println(err);
	// ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
	Serial.println("Started!");
}
