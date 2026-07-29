#ifndef BOARD_CFG_h
#define BOARD_CFG_h

#include <stdint.h>

#include "ADS131M0x_reg.h"
#include <soc/gpio_num.h>

struct FactoryResetCfg {
	gpio_num_t pin;
	bool activeLevelHi;

	constexpr bool connected() const { return pin != GPIO_NUM_NC; }
};

struct I2cConnectCfg {
	gpio_num_t masterSdaIo;
	gpio_num_t masterSclIo;

	constexpr bool connected() const {
		return (masterSdaIo != GPIO_NUM_NC) && (masterSclIo != GPIO_NUM_NC);
	}
};

struct TMP118SensorCfg {
	I2cConnectCfg i2c;
	char TMP118SubType; // TMP118A/B/C/D
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
	TMP118SensorCfg temperatureSensor;
	FactoryResetCfg factoryReset;
	AdcCfg<N> adc;
};

constexpr I2cConnectCfg i2cNotConnected{
    .masterSdaIo = GPIO_NUM_NC,
    .masterSclIo = GPIO_NUM_NC,
};

constexpr FactoryResetCfg resetNotConnected{
    .pin           = GPIO_NUM_NC,
    .activeLevelHi = false,
};

constexpr AdcHwConnect adcHwConnect1{
    .cs    = GPIO_NUM_13,
    .drdy  = GPIO_NUM_12,
    .reset = GPIO_NUM_14,
};

constexpr AdcSpiConnect adcSpiConnect1{
    .clock = GPIO_NUM_11,
    .miso  = GPIO_NUM_10,
    .mosi  = GPIO_NUM_9,
};

// Shift pins
constexpr AdcHwConnect adcHwConnect2{
    .cs    = GPIO_NUM_21,
    .drdy  = GPIO_NUM_14,
    .reset = GPIO_NUM_47,
};

// Shift pins
constexpr AdcSpiConnect adcSpiConnect2{
    .clock = GPIO_NUM_13,
    .miso  = GPIO_NUM_12,
    .mosi  = GPIO_NUM_11,
};

// V3.0.0 hardware
constexpr K3BoardCfg<4> boardv300{
    .name = "v300",
    .temperatureSensor =
        {
            .i2c           = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect1,
            .spiConnect = adcSpiConnect1,
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
    .name = "v400",
    .temperatureSensor =
        {
            .i2c           = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect1,
            .spiConnect = adcSpiConnect1,
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
                    // NOTE - gain should be at 1x-4x to be within
                    // datasheet max allowed V requirements
                    ADS131M0xReg::CHANNEL_PGA_4,
                    ADS131M0xReg::CHANNEL_PGA_4,
                    ADS131M0xReg::CHANNEL_PGA_4,
                    ADS131M0xReg::CHANNEL_PGA_4,
                },
            .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
            .osr       = ADS131M0xReg::OSR_4096,
        },
};

// V5.0.0 hardware
constexpr K3BoardCfg<4> boardv500{
    .name = "v500",
    .temperatureSensor =
        {
            .i2c           = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect1,
            .spiConnect = adcSpiConnect1,
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
                    // NOTE - gain should be at 1x-4x to be within
                    // datasheet max allowed V requirements
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
    .name = "v600L",
    .temperatureSensor =
        {
            .i2c           = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect1,
            .spiConnect = adcSpiConnect1,
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
                    ADS131M0xReg::CHANNEL_PGA_32,
                    ADS131M0xReg::CHANNEL_PGA_32,
                    ADS131M0xReg::CHANNEL_PGA_32,
                    ADS131M0xReg::CHANNEL_PGA_32,
                },
            .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
            .osr       = ADS131M0xReg::OSR_4096,
        },
};

// V6 Pro hardware
constexpr K3BoardCfg<8> boardv600_Pro{
    .name = "v600P",
    .temperatureSensor =
        {
            .i2c =
                {
                    .masterSdaIo = GPIO_NUM_46,
                    .masterSclIo = GPIO_NUM_3,
                },
            .TMP118SubType = 'A',
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect1,
            .spiConnect = adcSpiConnect1,
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
                    // NOTE - gain should be at 1x-4x to be within
                    // datasheet max allowed V requirements
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

// V7 Lite hardware
constexpr K3BoardCfg<4> boardv700_lite{
    .name = "v700L",
    .temperatureSensor =
        {
            .i2c           = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            .hwConnect  = adcHwConnect1,
            .spiConnect = adcSpiConnect1,
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
                    ADS131M0xReg::CHANNEL_PGA_32,
                    ADS131M0xReg::CHANNEL_PGA_32,
                    ADS131M0xReg::CHANNEL_PGA_32,
                    ADS131M0xReg::CHANNEL_PGA_32,
                },
            .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
            .osr       = ADS131M0xReg::OSR_4096,
        },
};

// V7 Pro hardware
constexpr K3BoardCfg<4> boardv700_Pro{
    .name = "v700P",
    .temperatureSensor =
        {
            .i2c =
                {
                    .masterSdaIo = GPIO_NUM_10,
                    .masterSclIo = GPIO_NUM_9,
                },
            .TMP118SubType = 'A',
        },
    .factoryReset = resetNotConnected,
    .adc =
        {
            // TODO SPI pins are now also different
            .hwConnect  = adcHwConnect2,
            .spiConnect = adcSpiConnect2,
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
                    // NOTE - gain should be at 1x-4x to be within
                    // datasheet max allowed V requirements
                    ADS131M0xReg::CHANNEL_PGA_1,
                    ADS131M0xReg::CHANNEL_PGA_1,
                    ADS131M0xReg::CHANNEL_PGA_1,
                    ADS131M0xReg::CHANNEL_PGA_1,
                },
            .powerMode = ADS131M0xReg::POWER_MODE_HIGH_RESOLUTION,
            .osr       = ADS131M0xReg::OSR_4096,
        },
};

#if CONFIG_DYNAMITE_HW_REV_V3
constexpr auto boardConfig{boardv300};
#elif CONFIG_DYNAMITE_HW_REV_V4
constexpr auto boardConfig{boardv400};
#elif CONFIG_DYNAMITE_HW_REV_V5
constexpr auto boardConfig{boardv500};
#elif CONFIG_DYNAMITE_HW_REV_V6_LITE
constexpr auto boardConfig{boardv600_lite};
#elif CONFIG_DYNAMITE_HW_REV_V6_PRO
constexpr auto boardConfig{boardv600_Pro};
#elif CONFIG_DYNAMITE_HW_REV_V7_LITE
constexpr auto boardConfig{boardv700_lite};
#elif CONFIG_DYNAMITE_HW_REV_V7_PRO
constexpr auto boardConfig{boardv700_Pro};
#else
#error No board configuration selected.
#endif

#endif // BOARD_CFG_h
