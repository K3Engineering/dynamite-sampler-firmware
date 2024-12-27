#include "ADS131M0x.h"

#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <io_pin_remap.h>

#include "adc_ble_interface.h"
#include "adc_proc.h"

#include "debug_pin.h"
#include <HardwareSerial.h>

static ADS131M0x    adc;
static SPIClass     spiADC(HSPI);
static TaskHandle_t adcReadTaskHandle = NULL;

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

	if (!bleAccess.deviceConnected)
		return;

	BleAdcFeedData toSend{.status = adcReading.status, .data = {}, .crc = adc.isCrcOk(&adcReading)};
	static_assert(sizeof(toSend.data) == sizeof(adcReading.data));
	memcpy(toSend.data, adcReading.data, sizeof(toSend.data));
	xStreamBufferSend(bleAccess.adcStreamBufferHandle, &toSend, sizeof(toSend), 0);
	// When the buffer is sufficiently large, time to send data.
	if (xStreamBufferBytesAvailable(bleAccess.adcStreamBufferHandle) >= ADC_FEED_CHUNK_SZ) {
		xTaskNotifyGive(bleAccess.bleAdcFeedPublisherTaskHandle);
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
	attachInterrupt(digitalPinToGPIONumber(PIN_DRDY), isrAdcDrdy, FALLING);

	// TODO figure out if you need to setup wake from sleep for gpio

	*(volatile bool *)setupDone = true;
	vTaskDelete(NULL);
}

void setupAdc(int core) {
	volatile bool done = false;
	xTaskCreatePinnedToCore(taskSetupAdc, "task_ADC_setup", 1024 * 5, (void *)&done, 1, NULL, core);
	while (!done)
		;

	// TODO figure out the memory stack required
	const UBaseType_t priority = 24; // Highest priority possible
	xTaskCreatePinnedToCore(taskAdcReadAndBuffer, "task_ADC_read", 1024 * 5, NULL, priority,
	                        &adcReadTaskHandle, core);
}
