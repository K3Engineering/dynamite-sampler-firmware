#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include <stdint.h>

#include "ADS131M0x_reg.h"

template <size_t N>
struct K3BoardCfg {
	static constexpr size_t NCHAN = N;
	char name[8];
	bool hasI2C;

	bool enable[NCHAN];
	uint16_t input[NCHAN];
	uint16_t pga[NCHAN];
	uint16_t powerMode;
	uint16_t osr;

	uint8_t translate[NCHAN];
	bool checkCRC;
};

// V3.0.0 hardware
constexpr K3BoardCfg<4> boardv300{
    .name   = "v300",
    .hasI2C = false,
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
    .translate = {0, 1, 2, 3},
    .checkCRC  = false,
};

// V4.0.0 hardware
constexpr K3BoardCfg<4> boardv400{
    .name   = "v400",
    .hasI2C = false,
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
    .translate = {0, 1, 2, 3},
    .checkCRC  = true,
};

// V5.0.0 hardware
constexpr K3BoardCfg<4> boardv500{
    .name   = "v500",
    .hasI2C = false,
    .enable =
        {
            true,
            true,
            true,
            false,
        },
    .input =
        {
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_INPUT_SHORTED,
        },
    .pga =
        {
            // NOTE - on this hardware revision, DIRECT gain
            // should be within 1x-4x to be within datasheet max allowed V requirements
            ADS131M0xReg::CHANNEL_PGA_1, // DIRECT, bottom connector
            ADS131M0xReg::CHANNEL_PGA_1, // OP AMP, bottom connector
            ADS131M0xReg::CHANNEL_PGA_1, // DIRECT, top connector
            ADS131M0xReg::CHANNEL_PGA_1, // OP AMP, top connector
        },
    .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
    .osr       = ADS131M0xReg::OSR_4096,
    .translate = {0, 1, 2, 3},
    .checkCRC  = false,
};

// V6 Lite hardware
constexpr K3BoardCfg<4> boardv600_lite{
    .name   = "v600L",
    .hasI2C = false,
    .enable =
        {
            true,
            true,
            true,
            true,
        },
    .input =
        {
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
        },
    .pga =
        {
            // NOTE - on this hardware revision, DIRECT gain
            // should be within 1x-4x to be within datasheet max allowed V requirements
            ADS131M0xReg::CHANNEL_PGA_32, // DIRECT, bottom connector
            ADS131M0xReg::CHANNEL_PGA_32, // OP AMP, bottom connector
            ADS131M0xReg::CHANNEL_PGA_32, // DIRECT, top connector
            ADS131M0xReg::CHANNEL_PGA_32, // OP AMP, top connector
        },
    .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
    .osr       = ADS131M0xReg::OSR_4096,
    .translate = {0, 1, 2, 3},
    .checkCRC  = false,
};

// V6 Pro hardware
constexpr K3BoardCfg<8> boardv600_Pro{
    .name   = "v600P",
    .hasI2C = true,
    .enable =
        {
            true,
            true,
            true,
            true,
            true,
            true,
            true,
            true,
        },
    .input =
        {
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
            ADS131M0xReg::INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS,
        },
    .pga =
        {
            // NOTE - on this hardware revision, DIRECT gain
            // should be within 1x-4x to be within datasheet max allowed V requirements
            ADS131M0xReg::CHANNEL_PGA_1,
            ADS131M0xReg::CHANNEL_PGA_1,
            ADS131M0xReg::CHANNEL_PGA_1,
            ADS131M0xReg::CHANNEL_PGA_1,
            ADS131M0xReg::CHANNEL_PGA_1,
            ADS131M0xReg::CHANNEL_PGA_1,
            ADS131M0xReg::CHANNEL_PGA_1,
            ADS131M0xReg::CHANNEL_PGA_1,
        },
    .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
    .osr       = ADS131M0xReg::OSR_4096,
    .translate = {1, 3, 5, 7, 0, 0, 0, 0},
    .checkCRC  = false,
};

#if (CONFIG_DYNAMITE_HW_REV_V3 == 1)
constexpr auto boardConfig{boardv300};
#elif (CONFIG_DYNAMITE_HW_REV_V4 == 1)
constexpr auto boardConfig{boardv400};
#elif (CONFIG_DYNAMITE_HW_REV_V5 == 1)
constexpr auto boardConfig{boardv500};
#elif (CONFIG_DYNAMITE_HW_REV_V6_LITE == 1)
constexpr auto boardConfig{boardv600_lite};
#elif (CONFIG_DYNAMITE_HW_REV_V6_PRO == 1)
constexpr auto boardConfig{boardv600_Pro};
#else
#error No board configuration selected.
#endif

#endif // ADS131M0x_CFG_h
