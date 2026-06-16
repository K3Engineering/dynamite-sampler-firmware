#include "ADS131M0x.h"

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <esp_log.h>

#include <endian.h>
#include <string.h>

#include "adc_ble_interface.h"
#include "adc_proc.h"

#include "ADS131M0x_cfg.h"
#include "ADS131M0x_reg.h"
#include "debug_pin.h"

constexpr char TAG[] = "ADC";

struct ADS131HwConfigData {
	uint16_t id;
	uint16_t status;
	uint16_t mode;
	uint16_t clock;
	uint16_t pga;
};

static DRAM_ATTR AdcClass adc;
static ADS131HwConfigData savedConfig;

bool startAdcAcquisition() { return adc.startAcquisition(); }

void stopAdcAcquisition() { adc.stopAcquisition(); }

const AdcConfigNetworkData getAdcConfig() {
	const ADS131HwConfigData *p = &savedConfig;
	return AdcConfigNetworkData{
	    .version = 1,
	    .id      = htole16(p->id),
	    .status  = htole16(p->status),
	    .mode    = htole16(p->mode),
	    .clock   = htole16(p->clock),
	    .pga     = htole16(p->pga),
	};
}

static void logADS131M0xConfig(const ADS131HwConfigData *cfg) {
	ESP_LOGI(TAG, "<REGISTERS>");
	ESP_LOGI(TAG, "ID 131M0x%X", (cfg->id >> 8) & 0x0F);
	ESP_LOGI(TAG, "STATUS 0x%04X", cfg->status);
	ESP_LOGI(TAG, "MODE 0x%04X", cfg->mode);
	const uint16_t clock = cfg->clock;
	ESP_LOGI(TAG, "CLOCK 0x%04X", cfg->clock);
	ESP_LOGI(TAG, " POWER MODE %u", clock & ADS131M0xReg::REGMASK_CLOCK_PWR);
	ESP_LOGI(TAG, " OSR %u", 128 << ((clock & ADS131M0xReg::REGMASK_CLOCK_OSR) >> 2));
	ESP_LOGI(TAG, " Turbo %c", (clock & ADS131M0xReg::REGMASK_CLOCK_TBM) ? 'Y' : 'N');
	ESP_LOGI(TAG, " Ch enabled 0x%X", (clock >> 8) & 0xF);
	uint16_t pga = cfg->pga;
	for (size_t i = 0; i < 4; ++i) {
		ESP_LOGI(TAG, "GAIN ch %u = %u", i, 1 << (pga & ADS131M0xReg::REGMASK_GAIN_PGAGAIN0));
		pga >>= 4;
	};
}

static inline void copyAdcToLE24(void *dst, const void *src) {
	static_assert(DYNAMITE_NET_BYTE_ORDER != AdcClass::RawOutput::SAMPLE_BYTE_ORDER);
	static_assert(AdcFeedNetworkData::AdcSample::BYTES_PER_SAMPLE == AdcClass::DATA_WORD_LENGTH);
	static_assert(AdcClass::DATA_WORD_LENGTH == 3, "Assumes 24-bit ADC sample");

	((uint8_t *)dst)[0] = ((const uint8_t *)src)[2];
	((uint8_t *)dst)[1] = ((const uint8_t *)src)[1];
	((uint8_t *)dst)[2] = ((const uint8_t *)src)[0];
}

static AdcFeedNetworkData IRAM_ATTR adcToNetwork(const AdcClass::RawOutput *adc) {
	AdcFeedNetworkData net;
	static_assert(AdcFeedNetworkData::NUM_CHAN <= boardConfig.NCHAN);
	for (size_t i = 0; i < AdcFeedNetworkData::NUM_CHAN; ++i) {
		size_t src_idx;
		if constexpr (AdcFeedNetworkData::NUM_CHAN == boardConfig.NCHAN) {
			src_idx = i;
		} else {
			src_idx = boardConfig.translate[i];
		}
		copyAdcToLE24(net.chan + i, adc->data + AdcClass::DATA_WORD_LENGTH * src_idx);
	}
	return net;
}

// Task that handles calling the read adc function and placing the values in the buffer.
static void IRAM_ATTR taskAdcReadAndBuffer(void *) {
	static constexpr size_t N_SAMPLES = AdcFeedNetworkPacket::NUM_SAMPLES;
	AdcFeedNetworkData toSend[N_SAMPLES];

	while (true) {
		// Wait until ISR notifies this task. Normally numNotifications == 1,
		// numNotifications > 1 in case we cannot keep up with adc and have some data lost.
		uint32_t numNotifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (0 == numNotifications) [[unlikely]] {
			continue;
		}
		// Read ADC values. Place them in StreamBuffer. Notify the BLE task
		const size_t idx = adc.getReadyBatchStartIdx();
		for (size_t n = 0; n < N_SAMPLES; ++n) {
			toSend[n] = adcToNetwork(adc.rawReadADC(idx + n));
		}
		if (sizeof(toSend) != xStreamBufferSend(adcStreamBufferHandle, toSend, sizeof(toSend), 0))
		    [[unlikely]] {
			ESP_LOGE(TAG, "xStreamBufferSend failed");
		}
	}
	vTaskDelete(NULL);
}

static void configureAdc() {
	static_assert(boardConfig.NCHAN == adc.NUM_CHANNELS);
	for (uint8_t chan = 0; chan < adc.NUM_CHANNELS; ++chan) {
		adc.setChannelEnable(chan, boardConfig.enable[chan]);
		adc.setChannelInputSelection(chan, boardConfig.input[chan]);
		adc.setChannelPGA(chan, boardConfig.pga[chan]);
	}
	adc.setPowerMode(boardConfig.powerMode);
	adc.setOsr(boardConfig.osr);

	savedConfig = {
	    .id     = adc.readID(),
	    .status = adc.readSTATUS(),
	    .mode   = adc.readMODE(),
	    .clock  = adc.readCLOCK(),
	    .pga    = adc.readPGA(),
	};
	logADS131M0xConfig(&savedConfig);
}

static void taskSetupAdc(void *setupDone) {
	ESP_LOGI(TAG, "setting up adc on core: %u", esp_cpu_get_core_id());
	constexpr gpio_num_t PIN_NUM_CLK   = GPIO_NUM_11;
	constexpr gpio_num_t PIN_NUM_MISO  = GPIO_NUM_10;
	constexpr gpio_num_t PIN_NUM_MOSI  = GPIO_NUM_9;
	constexpr gpio_num_t PIN_DRDY      = GPIO_NUM_12;
	constexpr gpio_num_t PIN_ADC_RESET = GPIO_NUM_14;
	constexpr gpio_num_t PIN_CS_ADC    = GPIO_NUM_13;

	// TODO figure out if you need to setup wake from sleep for gpio
	gpio_set_direction(PIN_DEBUG_TOP, GPIO_MODE_OUTPUT);
	gpio_set_direction(PIN_DEBUG_BOT, GPIO_MODE_OUTPUT);

	adc.init(PIN_CS_ADC, PIN_DRDY, PIN_ADC_RESET, SPI3_HOST, PIN_NUM_CLK, PIN_NUM_MISO,
	         PIN_NUM_MOSI);

	if (adc.resetAdcHw()) {
		configureAdc();
		*(volatile bool *)setupDone = true;
	} else {
		startupDiagnosticIsOk = false;
		// TODO: review init errors handling
	}
	vTaskDelete(NULL);
}

void setupAdc(int core) {
	volatile bool done = false;
	xTaskCreatePinnedToCore(taskSetupAdc, "task_ADC_setup", 1024 * 5, (void *)&done, 1, NULL, core);
	while (!done)
		vTaskDelay(10);

	// TODO figure out the memory stack required
	TaskHandle_t adcReadTaskHandle = 0;
	const UBaseType_t priority     = 24; // Highest priority possible
	xTaskCreatePinnedToCore(taskAdcReadAndBuffer, "task_ADC_read", 1024 * 4, NULL, priority,
	                        &adcReadTaskHandle, core);
	assert(adcReadTaskHandle);
	adc.setWakeupTask(adcReadTaskHandle, AdcFeedNetworkPacket::NUM_SAMPLES);
}
