#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include "ADS131M0x.h"

struct ADS131Cfg {
	uint8_t pga[4];
	uint8_t powerMode;
	uint8_t osr;
};

constexpr ADS131Cfg ads131UserConfig_boardv300 = {
    .pga =
        {
            // comments are valid for v4.0.0 hardware
            CHANNEL_PGA_1, // DIRECT, bottom connector
            CHANNEL_PGA_4, // OP AMP, bottom connector
            CHANNEL_PGA_4, // OP AMP, top connector
            CHANNEL_PGA_1, // DIRECT, top connector
        },
    .powerMode = POWER_MODE_HIGH_RESOLUTION,
    .osr       = OSR_4096,
};

constexpr ADS131Cfg ads131UserConfig_boardv400 = {
    .pga =
        {
            // comments are valid for v4.0.0 hardware
            CHANNEL_PGA_32, // DIRECT, bottom connector
            CHANNEL_PGA_4,  // OP AMP, bottom connector
            CHANNEL_PGA_32, // DIRECT, top connector
            CHANNEL_PGA_4,  // OP AMP, top connector
        },
    .powerMode = POWER_MODE_HIGH_RESOLUTION,
    .osr       = OSR_4096,
};

constexpr ADS131Cfg ads131UserConfig = ads131UserConfig_boardv400;

#endif // ADS131M0x_CFG_h
