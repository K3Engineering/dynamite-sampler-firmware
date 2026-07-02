#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include <stdint.h>

#include "ADS131M0x_reg.h"
#include <driver/i2c_types.h>
#include <soc/gpio_num.h>

struct I2cConnectCfg {
	gpio_num_t masterSdaIo;
	gpio_num_t masterSclIo;
	i2c_port_num_t masterPortNum;

	constexpr bool connected() const { return masterSdaIo != GPIO_NUM_NC; };
};

struct FactoryResetCfg {
	int pin;
	bool activeLevelHi;

	constexpr bool connected() const { return pin != -1; };
};

template <size_t N>
struct AdcCfg {
	static constexpr size_t NCHAN = N;
	bool enable[NCHAN];
	uint16_t input[NCHAN];
	uint16_t pga[NCHAN];
	uint16_t powerMode;
	uint16_t osr;
};

template <size_t N>
struct K3BoardCfg {
	char name[8];
	I2cConnectCfg i2c;
	FactoryResetCfg factoryReset;
	AdcCfg<N> adc;

	uint8_t translate[N];
};

constexpr I2cConnectCfg i2cNotConnected{
    .masterSdaIo   = GPIO_NUM_NC,
    .masterSclIo   = GPIO_NUM_NC,
    .masterPortNum = I2C_NUM_MAX,
};

constexpr FactoryResetCfg resetNotConnected{
    .pin           = -1,
    .activeLevelHi = false,
};

// V3.0.0 hardware
constexpr K3BoardCfg<4> boardv300{
    .name         = "v300",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
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
        },
    .translate = {0, 1, 2, 3},
};

// V4.0.0 hardware
constexpr K3BoardCfg<4> boardv400{
    .name         = "v400",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
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
        },
    .translate = {0, 1, 2, 3},
};

// V5.0.0 hardware
constexpr K3BoardCfg<4> boardv500{
    .name         = "v500",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
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
        },
    .translate = {0, 1, 2, 3},
};

// V6 Lite hardware
constexpr K3BoardCfg<4> boardv600_lite{
    .name         = "v600L",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
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
        },
    .translate = {0, 1, 2, 3},
};

// V6 Pro hardware
constexpr K3BoardCfg<8> boardv600_Pro{
    .name = "v600P",
    .i2c =
        {
            .masterSdaIo   = GPIO_NUM_46,
            .masterSclIo   = GPIO_NUM_3,
            .masterPortNum = I2C_NUM_0,
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
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
                    // should be within 1x-4x to be within datasheet max allowed V
                    // requirements
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
        },
    .translate = {1, 3, 5, 7, 0, 0, 0, 0},
};

struct GlobalSettings {
	bool checkAdcCrc;
};

constexpr GlobalSettings globalSettings{
    .checkAdcCrc = true,
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
