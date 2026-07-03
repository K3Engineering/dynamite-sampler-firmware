#ifndef ADS131M0x_CFG_h
#define ADS131M0x_CFG_h

#include <stdint.h>

#include "ADS131M0x_reg.h"
#include <soc/gpio_num.h>

struct FactoryResetCfg {
	int pin;
	bool activeLevelHi;

	constexpr bool connected() const { return pin != -1; };
};

struct I2cConnectCfg {
	gpio_num_t masterSdaIo;
	gpio_num_t masterSclIo;

	constexpr bool connected() const { return masterSdaIo != GPIO_NUM_NC; };
};

struct AdcHwConnect {
	gpio_num_t cs;
	gpio_num_t drdy;
	gpio_num_t reset;
};

struct AdcSpiConnect {
	gpio_num_t clock;
	gpio_num_t miso;
	gpio_num_t mosi;
};

template <size_t N>
struct AdcCfg {
	static constexpr size_t NCHAN = N;
	AdcHwConnect hwConnect;
	AdcSpiConnect spiConnect;
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
};

constexpr I2cConnectCfg i2cNotConnected{
    .masterSdaIo = GPIO_NUM_NC,
    .masterSclIo = GPIO_NUM_NC,
};

constexpr FactoryResetCfg resetNotConnected{
    .pin           = -1,
    .activeLevelHi = false,
};

constexpr AdcHwConnect adcHwConnect{
    .cs    = GPIO_NUM_13,
    .drdy  = GPIO_NUM_12,
    .reset = GPIO_NUM_14,
};

constexpr AdcSpiConnect adcSpiConnect{
    .clock = GPIO_NUM_11,
    .miso  = GPIO_NUM_10,
    .mosi  = GPIO_NUM_9,
};

// V3.0.0 hardware
constexpr K3BoardCfg<4> boardv300{
    .name         = "v300",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect,
            .spiConnect = adcSpiConnect,
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
};

// V4.0.0 hardware
constexpr K3BoardCfg<4> boardv400{
    .name         = "v400",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect,
            .spiConnect = adcSpiConnect,
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
};

// V5.0.0 hardware
constexpr K3BoardCfg<4> boardv500{
    .name         = "v500",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect,
            .spiConnect = adcSpiConnect,
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
};

// V6 Lite hardware
constexpr K3BoardCfg<4> boardv600_lite{
    .name         = "v600L",
    .i2c          = i2cNotConnected,
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect,
            .spiConnect = adcSpiConnect,
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
};

// V6 Pro hardware
constexpr K3BoardCfg<8> boardv600_Pro{
    .name = "v600P",
    .i2c =
        {
            .masterSdaIo = GPIO_NUM_46,
            .masterSclIo = GPIO_NUM_3,
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect,
            .spiConnect = adcSpiConnect,
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
