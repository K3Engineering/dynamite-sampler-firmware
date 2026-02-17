#include "ADS131M0x.h"

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <endian.h>
#include <string.h>

#include "adc_ble_interface.h"
#include "adc_proc.h"

#include "ADS131M0x_cfg.h"
#include "debug_pin.h"

constexpr char TAG[] = "ADC";

static AdcClass adc;
static TaskHandle_t adcReadTaskHandle = NULL;

static void logADS131M0xConfig(const ADS131M0x::ConfigData *cfg) {
	ESP_LOGI(TAG, "<REGISTERS>");
	ESP_LOGI(TAG, "ID 0x%X", cfg->id >> 8);
	ESP_LOGI(TAG, "STATUS 0x%04X", cfg->status);
	ESP_LOGI(TAG, "MODE 0x%04X", cfg->mode);
	const uint16_t clock = cfg->clock;
	ESP_LOGI(TAG, "CLOCK 0x%04X", cfg->clock);
	ESP_LOGI(TAG, " POWER MODE %u", clock & 0x03);
	ESP_LOGI(TAG, " OSR %u", 128 << ((clock >> 2) & 0x07));
	ESP_LOGI(TAG, " Turbo %c", (clock & 0x20) ? 'Y' : 'N');
	ESP_LOGI(TAG, " Ch enabled 0x%X", (clock >> 8) & 0xF);
	uint16_t pga = cfg->pga;
	for (size_t i = 0; i < sizeof(pga) * 2; ++i) {
		ESP_LOGI(TAG, "GAIN ch %u = %u", i, 1 << (pga & 0x07));
		pga >>= 4;
	};
}

const AdcConfigNetworkData getAdcConfig() {
	const ADS131M0x::ConfigData *p = adc.getConfig();
	return AdcConfigNetworkData{
	    .version = 1,
	    .id      = htole16(p->id),
	    .status  = htole16(p->status),
	    .mode    = htole16(p->mode),
	    .clock   = htole16(p->clock),
	    .pga     = htole16(p->pga),
	};
}

void startAdc() { adc.startAdc(); }
void stopAdc() { adc.stopAdc(); }

static inline void requestTaskSwitchFromISR(BaseType_t needSwitch, void *ptr) {
#if (CONFIG_MOCK_ADC == 1)
	*(bool *)ptr = (needSwitch == pdTRUE);
#else
	portYIELD_FROM_ISR(needSwitch);
#endif
}

// When we flag a piece of code with the IRAM_ATTR attribute, the compiled code
// is placed in the ESP32’s Internal RAM (IRAM). Otherwise the code is kept in
// Flash. And Flash on ESP32 is much slower than internal RAM.
static void IRAM_ATTR isrAdcDrdy(void *param) {
	// unblock the task that will read the ADC & handle putting in the buffer
	if (adcReadTaskHandle != NULL) {
		BaseType_t taskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(adcReadTaskHandle, &taskWoken);
		requestTaskSwitchFromISR(taskWoken, param);
	}
}

static inline void copy_adc_tole24(void *dst, const void *src) {
	static_assert(DYNAMITE_NET_BYTE_ORDER != AdcClass::RawOutput::SAMPLE_BYTE_ORDER);
	static_assert(AdcFeedNetworkData::AdcSample::BYTES_PER_SAMPLE == AdcClass::DATA_WORD_LENGTH);
	static_assert(AdcClass::DATA_WORD_LENGTH == 3, "Assumes 24-bit ADC samle");

	((uint8_t *)dst)[0] = ((const uint8_t *)src)[2];
	((uint8_t *)dst)[1] = ((const uint8_t *)src)[1];
	((uint8_t *)dst)[2] = ((const uint8_t *)src)[0];
}

static AdcFeedNetworkData adcToNetwork(const AdcClass::RawOutput *adc) {
	AdcFeedNetworkData net;
	for (size_t i = 0; i < AdcFeedNetworkData::NUM_CHAN; ++i) {
		copy_adc_tole24(net.chan + i, adc->data + AdcClass::DATA_WORD_LENGTH * i);
	}
	return net;
}
// Read ADC values. If BLE device is connected, place them in the buffer.
// When accumulated enough, notify the ble task
static void adcReadAndBuffer() {
	const AdcClass::RawOutput *adcReading = adc.rawReadADC();

	if (!bleAccess.clientSubscribed) {
		return;
	}
	AdcFeedNetworkData toSend = adcToNetwork(adcReading);
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
	ESP_LOGI(TAG, "setting up adc on core: %u", esp_cpu_get_core_id());
	constexpr gpio_num_t PIN_NUM_CLK   = GPIO_NUM_11;
	constexpr gpio_num_t PIN_NUM_MISO  = GPIO_NUM_10;
	constexpr gpio_num_t PIN_NUM_MOSI  = GPIO_NUM_9;
	constexpr gpio_num_t PIN_DRDY      = GPIO_NUM_12;
	constexpr gpio_num_t PIN_ADC_RESET = GPIO_NUM_14;
	constexpr gpio_num_t PIN_CS_ADC    = GPIO_NUM_13;

	gpio_set_direction(PIN_DEBUG_TOP, GPIO_MODE_OUTPUT);
	gpio_set_direction(PIN_DEBUG_BOT, GPIO_MODE_OUTPUT);

	adc.init(PIN_CS_ADC, PIN_DRDY, PIN_ADC_RESET);
	adc.setupAccess(SPI3_HOST, SPI_MASTER_FREQ_20M, PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI);

	adc.reset();

	for (uint8_t chan = 0; chan < 4; ++chan) {
		adc.setInputChannelSelection(chan, INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS);
	}
	adc.setPGA(ads131UserConfig.pga[0], ads131UserConfig.pga[1], ads131UserConfig.pga[2],
	           ads131UserConfig.pga[3]);
	adc.setPowerMode(ads131UserConfig.powerMode);
	adc.setOsr(ads131UserConfig.osr);

	adc.stashConfig();
	logADS131M0xConfig(adc.getConfig());

	adc.attachISR(isrAdcDrdy);

	// TODO figure out if you need to setup wake from sleep for gpio

	*(volatile bool *)setupDone = true;
	vTaskDelete(NULL);
}

void setupAdc(int core) {
	volatile bool done = false;
	xTaskCreatePinnedToCore(taskSetupAdc, "task_ADC_setup", 1024 * 5, (void *)&done, 1, NULL, core);
	while (!done)
		vTaskDelay(10);

	// TODO figure out the memory stack required
	const UBaseType_t priority = 24; // Highest priority possible
	xTaskCreatePinnedToCore(taskAdcReadAndBuffer, "task_ADC_read", 1024 * 5, NULL, priority,
	                        &adcReadTaskHandle, core);
}
