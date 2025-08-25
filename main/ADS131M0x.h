#ifndef ADS131M0x_h
#define ADS131M0x_h

#include <stddef.h>
#include <stdint.h>

#include <driver/spi_master.h>
#include <soc/gpio_num.h>

#include "dynamite_sampler_api.h"

#define DRDY_STATE_LOGIC_HIGH 0 // DEFAULS
#define DRDY_STATE_HI_Z       1

#define POWER_MODE_VERY_LOW_POWER  0
#define POWER_MODE_LOW_POWER       1
#define POWER_MODE_HIGH_RESOLUTION 2 // DEFAULT

#define CHANNEL_PGA_1   0
#define CHANNEL_PGA_2   1
#define CHANNEL_PGA_4   2
#define CHANNEL_PGA_8   3
#define CHANNEL_PGA_16  4
#define CHANNEL_PGA_32  5
#define CHANNEL_PGA_64  6
#define CHANNEL_PGA_128 7

#define INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS      0 // Default
#define INPUT_CHANNEL_MUX_INPUT_SHORTED           1
#define INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL 2
#define INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL 3

#define OSR_128   0 // 32 kSPS
#define OSR_256   1 // 16 kSPS
#define OSR_512   2 // 8 kSPS
#define OSR_1024  3 // default, 4 kSPS
#define OSR_2048  4 // 2 kSPS
#define OSR_4096  5 // 1 kSPS
#define OSR_8192  6 // 0.5 kSPS
#define OSR_16384 7 // 0.25 kSPS

// Commands
#define CMD_NULL      0x0000 // This command gives the STATUS REGISTER
#define CMD_RESET     0x0011
#define CMD_STANDBY   0x0022
#define CMD_WAKEUP    0x0033
#define CMD_LOCK      0x0555
#define CMD_UNLOCK    0x0655
#define CMD_READ_REG  0xA000 // 101a aaaa annn nnnn   a=adress  n=num  regis-1
#define CMD_WRITE_REG 0x6000

// Responses
#ifdef IS_M02
#define RSP_RESET_OK 0xFF22
#else
#define RSP_RESET_OK 0xFF24
#endif
#define RSP_RESET_NOK 0x0011
#define RSP_WRITE_REG 0x4000

// Registers Read Only
#define REG_ID     0x00
#define REG_STATUS 0x01

// Registers Global Settings across channels
#define REG_MODE    0x02
#define REG_CLOCK   0x03
#define REG_GAIN    0x04
#define REG_CFG     0x06
#define THRSHLD_MSB 0x07
#define THRSHLD_LSB 0x08

// Registers Channel 0 Specific
#define REG_CH0_CFG      0x09
#define REG_CH0_OCAL_MSB 0x0A
#define REG_CH0_OCAL_LSB 0x0B
#define REG_CH0_GCAL_MSB 0x0C
#define REG_CH0_GCAL_LSB 0x0D

// Registers Channel 1 Specific
#define REG_CH1_CFG      0x0E
#define REG_CH1_OCAL_MSB 0x0F
#define REG_CH1_OCAL_LSB 0x10
#define REG_CH1_GCAL_MSB 0x11
#define REG_CH1_GCAL_LSB 0x12

// Registers Channel 2 Specific
#define REG_CH2_CFG      0x13
#define REG_CH2_OCAL_MSB 0x14
#define REG_CH2_OCAL_LSB 0x15
#define REG_CH2_GCAL_MSB 0x16
#define REG_CH2_GCAL_LSB 0x17

// Registers Channel 3 Specific
#define REG_CH3_CFG      0x18
#define REG_CH3_OCAL_MSB 0x19
#define REG_CH3_OCAL_LSB 0x1A
#define REG_CH3_GCAL_MSB 0x1B
#define REG_CH3_GCAL_LSB 0x1C

// Registers MAP CRC
#define REG_MAP_CRC 0x3E

// ------------------------------------------------------------------------------------

// Mask READ_REG
#define REGMASK_CMD_READ_REG_ADDRESS 0x1F80
#define REGMASK_CMD_READ_REG_BYTES   0x007F

// Mask Register STATUS
#define REGMASK_STATUS_LOCK     0x8000
#define REGMASK_STATUS_RESYNC   0x4000
#define REGMASK_STATUS_REGMAP   0x2000
#define REGMASK_STATUS_CRC_ERR  0x1000
#define REGMASK_STATUS_CRC_TYPE 0x0800
#ifdef IS_M02
#define REGMASK_STATUS_RESET 0x0200
#else
#define REGMASK_STATUS_RESET 0x0400
#endif
#define REGMASK_STATUS_WLENGTH 0x0300
#define REGMASK_STATUS_DRDY3   0x0008
#define REGMASK_STATUS_DRDY2   0x0004
#define REGMASK_STATUS_DRDY1   0x0002
#define REGMASK_STATUS_DRDY0   0x0001

// Mask Register MODE
#define REGMASK_MODE_REG_CRC_EN 0x2000
#define REGMASK_MODE_RX_CRC_EN  0x1000
#define REGMASK_MODE_CRC_TYPE   0x0800
#define REGMASK_MODE_RESET      0x0400
#define REGMASK_MODE_WLENGTH    0x0300
#define REGMASK_MODE_TIMEOUT    0x0010
#define REGMASK_MODE_DRDY_SEL   0x000C
#define REGMASK_MODE_DRDY_HiZ   0x0002
#define REGMASK_MODE_DRDY_FMT   0x0001

// Mask Register CLOCK
#define REGMASK_CLOCK_CH3_EN 0x0800
#define REGMASK_CLOCK_CH2_EN 0x0400
#define REGMASK_CLOCK_CH1_EN 0x0200
#define REGMASK_CLOCK_CH0_EN 0x0100
#define REGMASK_CLOCK_OSR    0x001C
#define REGMASK_CLOCK_PWR    0x0003

// Mask Register GAIN
#define REGMASK_GAIN_PGAGAIN3 0x7000
#define REGMASK_GAIN_PGAGAIN2 0x0700
#define REGMASK_GAIN_PGAGAIN1 0x0070
#define REGMASK_GAIN_PGAGAIN0 0x0007

// Mask Register CFG
#define REGMASK_CFG_GC_DLY   0x1E00
#define REGMASK_CFG_GC_EN    0x0100
#define REGMASK_CFG_CD_ALLCH 0x0080
#define REGMASK_CFG_CD_NUM   0x0070
#define REGMASK_CFG_CD_LEN   0x000E
#define REGMASK_CFG_CD_EN    0x0001

// Mask Register THRSHLD_LSB
#define REGMASK_THRSHLD_LSB_CD_TH_LSB 0xFF00
#define REGMASK_THRSHLD_LSB_DCBLOCK   0x000F

// Mask Register CHX_CFG
#define REGMASK_CHX_CFG_PHASE       0xFFC0
#define REGMASK_CHX_CFG_DCBLKX_DIS0 0x0004
#define REGMASK_CHX_CFG_MUX         0x0003

// Mask Register CHX_OCAL_LSB
#define REGMASK_CHX_OCAL0_LSB 0xFF00

// Mask Register CHX_GCAL_LSB
#define REGMASK_CHX_GCAL0_LSB 0xFF00

class ADS131M0x {
  public:
	static constexpr size_t NUM_CHANNELS_ENABLED = 4;
	static constexpr size_t DATA_WORD_LENGTH     = 3; // in bytes
	static constexpr size_t ADC_READ_DATA_SIZE =
	    (1 + NUM_CHANNELS_ENABLED + 1) * DATA_WORD_LENGTH; // status, channels, CRC

#pragma pack(push, 1)
	struct AdcRawOutput {
		static constexpr size_t SAMPLE_BYTE_ORDER = __ORDER_BIG_ENDIAN__;

		uint16_t status;
		uint8_t  status_unused;
		uint8_t  data[DATA_WORD_LENGTH * NUM_CHANNELS_ENABLED];
		uint16_t crc;
		uint8_t  crc_unused;
	};
#pragma pack(pop)
	static_assert(sizeof(AdcRawOutput) == ADC_READ_DATA_SIZE);

	void init(gpio_num_t cs_pin, gpio_num_t drdy_pin, gpio_num_t reset_pin);
	void setupAccess(spi_host_device_t spiDevice, int spi_clock_speed, gpio_num_t clk_pin,
	                 gpio_num_t miso_pin, gpio_num_t mosi_pin);
	void reset();
	bool setPowerMode(uint8_t powerMode);
	bool setChannelPGA(uint8_t channel, uint16_t pga);
	bool setPGA(uint8_t pgaChan0, uint8_t pgaChan1, uint8_t pgaChan2, uint8_t pgaChan3);
	bool setInputChannelSelection(uint8_t channel, uint8_t input);
	bool setOsr(uint16_t osr);

	typedef void (*AdcISR)(void *);
	void attachISR(AdcISR isr);
	void enableAdcInterrupt();
	void disableAdcInterrupt();

	const AdcRawOutput *rawReadADC();

	void                        stashConfig();
	const AdcConfigNetworkData *getConfig() const { return &savedConfig; }

	static bool isCrcOk(const AdcRawOutput *data);

  private:
	uint16_t readRegister(uint8_t address);

	bool writeRegister(uint8_t address, uint16_t value);
	bool writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask);

	uint16_t readID();
	uint16_t readSTATUS();
	uint16_t readMODE();
	uint16_t readCLOCK();
	uint16_t readPGA();

	spi_device_handle_t spiHandle;
	spi_transaction_t   transDesc;

	gpio_num_t csPin;
	gpio_num_t drdyPin;
	gpio_num_t resetPin;

	AdcConfigNetworkData savedConfig;

	// static for simplicity,
	// should be allocated per instance with MALLOC_CAP_DMA
	static AdcRawOutput spi2adc;
	static AdcRawOutput adc2spi;
};

#if (CONFIG_MOCK_ADC == 1)

#include <driver/gptimer.h>

class MockAdc {
	gptimer_handle_t gptimer;

  public:
	static constexpr size_t NUM_CHANNELS_ENABLED = ADS131M0x::NUM_CHANNELS_ENABLED;
	static constexpr size_t DATA_WORD_LENGTH     = ADS131M0x::DATA_WORD_LENGTH; // in bytes
	static constexpr size_t ADC_READ_DATA_SIZE   = ADS131M0x::ADC_READ_DATA_SIZE;

	void init(gpio_num_t cs_pin, gpio_num_t drdy_pin, gpio_num_t reset_pin) {}
	void setupAccess(spi_host_device_t spiDevice, uint32_t spi_clock_speed, gpio_num_t clk_pin,
	                 gpio_num_t miso_pin, gpio_num_t mosi_pin) {}
	void reset() {}
	bool setChannelPGA(uint8_t channel, uint16_t pga) { return true; }
	bool setPGA(uint8_t pgaChan0, uint8_t pgaChan1, uint8_t pgaChan2, uint8_t pgaChan3) {
		return true;
	}
	bool setPowerMode(uint8_t powerMode) { return true; }
	bool setInputChannelSelection(uint8_t channel, uint8_t input) { return true; }
	bool setOsr(uint16_t osr) { return true; }

	void        stashConfigAsText() {}
	const char *getConfigAsText() const { return "MockAdc"; }

	uint16_t readID() { return 0; }
	uint16_t readSTATUS() { return 0; }
	uint16_t readMODE() { return 0; }
	uint16_t readCLOCK() { return 0; }
	uint16_t readPGA() { return 0; }

	typedef ADS131M0x::AdcISR       AdcISR;
	typedef ADS131M0x::AdcRawOutput AdcRawOutput;

	void attachISR(AdcISR isr);
	void enableAdcInterrupt();
	void disableAdcInterrupt();

	const AdcRawOutput *rawReadADC();

	static bool isCrcOk(const AdcRawOutput *data) { return true; };
};

typedef MockAdc AdcClass;
#else
typedef ADS131M0x AdcClass;
#endif

#endif
