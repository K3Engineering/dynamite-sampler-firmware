#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include "ADS131M0x_reg.h"

template <size_t NCHANNELS>
struct ADS131UserCfg {
	bool enable[NCHANNELS];
	uint16_t input[NCHANNELS];
	uint16_t pga[NCHANNELS];
	uint16_t powerMode;
	uint16_t osr;
};

using ADS131M04UserCfg = ADS131UserCfg<4>;

// V3.0.0 hardware
constexpr ADS131M04UserCfg ads131UserConfig_boardv300{
    .enable =
        {
            false,
            true,
            true,
            false,
        },
    .input =
        {
            ADS131M0xReg::INPUT_CHANNEL_MUX_INPUT_SHORTED,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_INPUT_SHORTED,
        },
    .pga =
        {
            ADS131M0xReg::CHANNEL_PGA_1, // DIRECT, bottom connector
            ADS131M0xReg::CHANNEL_PGA_4, // OP AMP, bottom connector
            ADS131M0xReg::CHANNEL_PGA_4, // OP AMP, top connector
            ADS131M0xReg::CHANNEL_PGA_1, // DIRECT, top connector
        },
    .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
    .osr       = ADS131M0xReg::OSR_4096,
};

// V4.0.0 hardware
constexpr ADS131M04UserCfg ads131UserConfig_boardv400{
    .enable =
        {
            false,
            true,
            true,
            false,
        },
    .input =
        {
            ADS131M0xReg::INPUT_CHANNEL_MUX_INPUT_SHORTED,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_INPUT_SHORTED,
        },
    .pga =
        {
            // NOTE - on this hardware revision, DIRECT gain
            // should be within 1x-4x to be within datasheet max allowed V requirements
            ADS131M0xReg::CHANNEL_PGA_4, // DIRECT, bottom connector
            ADS131M0xReg::CHANNEL_PGA_4, // OP AMP, bottom connector
            ADS131M0xReg::CHANNEL_PGA_4, // DIRECT, top connector
            ADS131M0xReg::CHANNEL_PGA_4, // OP AMP, top connector
        },
    .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
    .osr       = ADS131M0xReg::OSR_4096,
};

constexpr ADS131M04UserCfg ads131UserConfig{ads131UserConfig_boardv400};

#endif // ADS131M0x_CFG_h
