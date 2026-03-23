#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include "ADS131M0x.h"

struct ADS131Cfg {
	bool enable[ADS131M0x::NUM_CHANNELS];
	uint16_t input[ADS131M0x::NUM_CHANNELS];
	uint16_t pga[ADS131M0x::NUM_CHANNELS];
	uint16_t powerMode;
	uint16_t osr;
};

// V3.0.0 hardware
constexpr ADS131Cfg ads131UserConfig_boardv300 = {
    .enable =
        {
            false,
            true,
            true,
            false,
        },
    .input =
        {
            INPUT_CHANNEL_MUX_INPUT_SHORTED,
            INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            INPUT_CHANNEL_MUX_INPUT_SHORTED,
        },
    .pga =
        {
            CHANNEL_PGA_1, // DIRECT, bottom connector
            CHANNEL_PGA_4, // OP AMP, bottom connector
            CHANNEL_PGA_4, // OP AMP, top connector
            CHANNEL_PGA_1, // DIRECT, top connector
        },
    .powerMode = POWER_MODE_HIGH_RESOLUTION,
    .osr       = OSR_4096,
};

// V4.0.0 hardware
constexpr ADS131Cfg ads131UserConfig_boardv400 = {
    .enable =
        {
            false,
            true,
            true,
            false,
        },
    .input =
        {
            INPUT_CHANNEL_MUX_INPUT_SHORTED,
            INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            INPUT_CHANNEL_MUX_INPUT_SHORTED,
        },
    .pga =
        {
            // NOTE - on this hardware revision, DIRECT gain
            // should be within 1x-4x to be within datasheet max allowed V requirements
            CHANNEL_PGA_4, // DIRECT, bottom connector
            CHANNEL_PGA_4, // OP AMP, bottom connector
            CHANNEL_PGA_4, // DIRECT, top connector
            CHANNEL_PGA_4, // OP AMP, top connector
        },
    .powerMode = POWER_MODE_HIGH_RESOLUTION,
    .osr       = OSR_4096,
};

constexpr ADS131Cfg ads131UserConfig = ads131UserConfig_boardv400;

#endif // ADS131M0x_CFG_h
