#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include "ADS131M0x.h"

struct ADS131Cfg {
	uint8_t pga[4];
	uint8_t powerMode;
	uint8_t osr;
};

constexpr ADS131Cfg ads131UserConfig = {
    .pga =
        {
            CHANNEL_PGA_1,
            CHANNEL_PGA_4,
            CHANNEL_PGA_4,
            CHANNEL_PGA_1,
        },
    .powerMode = POWER_MODE_HIGH_RESOLUTION,
    .osr       = OSR_4096,
};

#endif // ADS131M0x_CFG_h
