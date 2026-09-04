#ifndef BOARD_CFG_h
#define BOARD_CFG_h

#include <stdint.h>
#include <string.h>

#include "ADS131M0x_reg.h"
#include <soc/gpio_num.h>

// With a single build for multiple boards, we can't have different reset pins per board
struct FactoryResetCfg {
	gpio_num_t pin;
	bool activeLevelHi;

	constexpr bool connected() const { return pin != GPIO_NUM_NC; }
};

constexpr FactoryResetCfg factoryResetCfg{
    .pin = GPIO_NUM_NC,
    .activeLevelHi = false,
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

struct AdcCfg {
	static constexpr size_t NCHAN = 4;
	AdcHwConnect hwConnect;
	AdcSpiConnect spiConnect;
	bool enable[NCHAN];
	uint16_t input[NCHAN];
	uint16_t pga[NCHAN];
	uint16_t powerMode;
	uint16_t osr;
};

struct BoardCfg {
	char name[8]; // matches the Factory `board_model` key and the DIS Hardware Revision char
	char marketingName[32];
	TMP118SensorCfg temperatureSensor;
	AdcCfg adc;
};

constexpr I2cConnectCfg i2cNotConnected{
    .masterSdaIo = GPIO_NUM_NC,
    .masterSclIo = GPIO_NUM_NC,
};

constexpr AdcHwConnect adcHwConnect1{
    .cs = GPIO_NUM_13,
    .drdy = GPIO_NUM_12,
    .reset = GPIO_NUM_14,
};

constexpr AdcSpiConnect adcSpiConnect1{
    .clock = GPIO_NUM_11,
    .miso = GPIO_NUM_10,
    .mosi = GPIO_NUM_9,
};

// Shift pins
constexpr AdcHwConnect adcHwConnect2{
    .cs = GPIO_NUM_21,
    .drdy = GPIO_NUM_14,
    .reset = GPIO_NUM_47,
};

// Shift pins
constexpr AdcSpiConnect adcSpiConnect2{
    .clock = GPIO_NUM_13,
    .miso = GPIO_NUM_12,
    .mosi = GPIO_NUM_11,
};

// V3.0.0 hardware
constexpr BoardCfg boardV300{
    .name = "v300",
    .marketingName = "prototype",
    .temperatureSensor =
        {
            .i2c = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .adc =
        {
            .hwConnect = adcHwConnect1,
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
            .osr = ADS131M0xReg::OSR_4096,
        },
};

// V4.0.0 hardware
constexpr BoardCfg boardV400{
    .name = "v400",
    .marketingName = "prototype",
    .temperatureSensor =
        {
            .i2c = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .adc =
        {
            .hwConnect = adcHwConnect1,
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
            .osr = ADS131M0xReg::OSR_4096,
        },
};

// V5.0.0 hardware
constexpr BoardCfg boardV500{
    .name = "v500",
    .marketingName = "prototype",
    .temperatureSensor =
        {
            .i2c = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .adc =
        {
            .hwConnect = adcHwConnect1,
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
            .osr = ADS131M0xReg::OSR_4096,
        },
};

// V6 Lite hardware
constexpr BoardCfg boardV600L{
    .name = "v600L",
    .marketingName = "prototype",
    .temperatureSensor =
        {
            .i2c = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .adc =
        {
            .hwConnect = adcHwConnect1,
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
            .osr = ADS131M0xReg::OSR_4096,
        },
};

// V6 Pro hardware is not supported due to it having 8 channels

// V7 Lite hardware
constexpr BoardCfg boardV700L{
    .name = "v700L",
    .marketingName = "Dynamite Sampler Lite Mk1",
    .temperatureSensor =
        {
            .i2c = i2cNotConnected,
            .TMP118SubType = 0,
        },
    .adc =
        {
            .hwConnect = adcHwConnect1,
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
            .osr = ADS131M0xReg::OSR_4096,
        },
};

// V7 Pro hardware
constexpr BoardCfg boardV700P{
    .name = "v700P",
    .marketingName = "Dynamite Sampler Pro Mk1",
    .temperatureSensor =
        {
            .i2c =
                {
                    .masterSdaIo = GPIO_NUM_10,
                    .masterSclIo = GPIO_NUM_9,
                },
            .TMP118SubType = 'A',
        },
    .adc =
        {
            .hwConnect = adcHwConnect2,
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
            .osr = ADS131M0xReg::OSR_4096,
        },
};

constexpr BoardCfg kBoardCfgs[] = {boardV300,  boardV400,  boardV500,
                                   boardV600L, boardV700L, boardV700P};

inline const BoardCfg *findBoardCfg(const char *name) {
	for (const BoardCfg &cfg : kBoardCfgs) {
		if (0 == strcmp(cfg.name, name)) {
			return &cfg;
		}
	}
	return nullptr;
}

#endif // BOARD_CFG_h
