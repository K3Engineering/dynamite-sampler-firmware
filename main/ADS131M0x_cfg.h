#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include <esp_log.h>

#include "ADS131M0x.h"
#include "sdkconfig.h"

struct ADS131Cfg {
	uint8_t pga[4];
	uint8_t powerMode;
	uint8_t osr;
};

constexpr ADS131Cfg ads131UserConfig = {
    .pga =
        {
            CHANNEL_PGA_1,
            CHANNEL_PGA_32,
            CHANNEL_PGA_32,
            CHANNEL_PGA_1,
        },
    .powerMode = POWER_MODE_HIGH_RESOLUTION,
    .osr       = OSR_4096,
};

static inline void ads131LogRegisterMap(AdcClass &adc) {
#ifdef CONFIG_MOCK_ADC
	constexpr char TAG[] = "MockADC";
#else
	constexpr char TAG[] = "ADS131";
#endif

	ESP_LOGI(TAG, "REGISTER: ID 0x%X", adc.readID() >> 8);
	ESP_LOGI(TAG, "REGISTER: STATUS 0x%X", adc.readSTATUS());
	ESP_LOGI(TAG, "REGISTER: MODE 0x%X", adc.readMODE());
	uint16_t clock = adc.readCLOCK();
	ESP_LOGI(TAG, "REGISTER: CLOCK");
	ESP_LOGI(TAG, "POWER MODE %u", clock & 0x03);
	ESP_LOGI(TAG, "OSR %u", 128 << ((clock >> 2) & 0x07));
	ESP_LOGI(TAG, "Turbo %c", (clock & 0x20) ? 'Y' : 'N');
	ESP_LOGI(TAG, "Ch enabled 0x%X", (clock >> 8) & 0x0F);
	uint16_t pga = adc.readPGA();
	ESP_LOGI(TAG, "REGISTER: GAIN");
	for (size_t i = 0; i < sizeof(pga) * 2; ++i) {
		ESP_LOGI(TAG, "GAIN ch %u = %u", i, 1 << (pga & 0x07));
		pga >>= 4;
	}
}

#endif // ADS131M0x_CFG_h
