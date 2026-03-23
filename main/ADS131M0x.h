#ifndef ADS131M0x_h
#define ADS131M0x_h

#include <stddef.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/spi_master.h>
#include <soc/gpio_num.h>

#include <esp_rom_lldesc.h>
#include <soc/spi_struct.h>

class ADS131M0x {
  public:
	static constexpr size_t NUM_CHANNELS = 4;
	static_assert(NUM_CHANNELS == 4, "Tested on ADS131M04 only");

	static constexpr uint16_t DRDY_STATE_LOGIC_HIGH = 0; // Default
	static constexpr uint16_t DRDY_STATE_HI_Z       = 1;

	static constexpr uint16_t POWER_MODE_VERY_LOW_POWER  = 0;
	static constexpr uint16_t POWER_MODE_LOW_POWER       = 1;
	static constexpr uint16_t POWER_MODE_HIGH_RESOLUTION = 2; // Default

	static constexpr uint16_t CHANNEL_PGA_1   = 0;
	static constexpr uint16_t CHANNEL_PGA_2   = 1;
	static constexpr uint16_t CHANNEL_PGA_4   = 2;
	static constexpr uint16_t CHANNEL_PGA_8   = 3;
	static constexpr uint16_t CHANNEL_PGA_16  = 4;
	static constexpr uint16_t CHANNEL_PGA_32  = 5;
	static constexpr uint16_t CHANNEL_PGA_64  = 6;
	static constexpr uint16_t CHANNEL_PGA_128 = 7;

	static constexpr uint16_t INPUT_CHANNEL_MUX_DEFAULT_INPUT_PINS      = 0; // Default
	static constexpr uint16_t INPUT_CHANNEL_MUX_INPUT_SHORTED           = 1;
	static constexpr uint16_t INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL = 2;
	static constexpr uint16_t INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL = 3;

	static constexpr uint16_t OSR_128   = 0; // 32 kSPS
	static constexpr uint16_t OSR_256   = 1; // 16 kSPS
	static constexpr uint16_t OSR_512   = 2; // 8 kSPS
	static constexpr uint16_t OSR_1024  = 3; // default, 4 kSPS
	static constexpr uint16_t OSR_2048  = 4; // 2 kSPS
	static constexpr uint16_t OSR_4096  = 5; // 1 kSPS
	static constexpr uint16_t OSR_8192  = 6; // 0.5 kSPS
	static constexpr uint16_t OSR_16384 = 7; // 0.25 kSPS

	// Commands
	static constexpr uint16_t CMD_NULL    = 0x0000; // This command gives the STATUS REGISTER
	static constexpr uint16_t CMD_RESET   = 0x0011;
	static constexpr uint16_t CMD_STANDBY = 0x0022;
	static constexpr uint16_t CMD_WAKEUP  = 0x0033;
	static constexpr uint16_t CMD_LOCK    = 0x0555;
	static constexpr uint16_t CMD_UNLOCK  = 0x0655;
	static constexpr uint16_t CMD_READ_REG =
	    0xA000; // 101a aaaa annn nnnn   a=adress  n=num  regis-1
	static constexpr uint16_t CMD_WRITE_REG = 0x6000;

	// Responses
	static constexpr uint16_t RSP_RESET_OK  = 0xFF20 + NUM_CHANNELS;
	static constexpr uint16_t RSP_RESET_NOK = 0x0011;
	static constexpr uint16_t RSP_WRITE_REG = 0x4000;

	// Registers Read Only
	static constexpr uint16_t REG_ID     = 0x00;
	static constexpr uint16_t REG_STATUS = 0x01;

	// Registers Global Settings across channels
	static constexpr uint16_t REG_MODE  = 0x02;
	static constexpr uint16_t REG_CLOCK = 0x03;
	static constexpr uint16_t REG_GAIN  = 0x04;
	// static constexpr uint16_t REG_GAIN2   = 0x05;
	static constexpr uint16_t REG_CFG     = 0x06;
	static constexpr uint16_t THRSHLD_MSB = 0x07;
	static constexpr uint16_t THRSHLD_LSB = 0x08;

	// Registers Channel 0 Specific
	static constexpr uint16_t REG_CH0_CFG      = 0x09;
	static constexpr uint16_t REG_CH0_OCAL_MSB = 0x0A;
	static constexpr uint16_t REG_CH0_OCAL_LSB = 0x0B;
	static constexpr uint16_t REG_CH0_GCAL_MSB = 0x0C;
	static constexpr uint16_t REG_CH0_GCAL_LSB = 0x0D;

	// Registers Channel 1 Specific
	static constexpr uint16_t REG_CH1_CFG      = 0x0E;
	static constexpr uint16_t REG_CH1_OCAL_MSB = 0x0F;
	static constexpr uint16_t REG_CH1_OCAL_LSB = 0x10;
	static constexpr uint16_t REG_CH1_GCAL_MSB = 0x11;
	static constexpr uint16_t REG_CH1_GCAL_LSB = 0x12;

	// Registers Channel 2 Specific
	static constexpr uint16_t REG_CH2_CFG      = 0x13;
	static constexpr uint16_t REG_CH2_OCAL_MSB = 0x14;
	static constexpr uint16_t REG_CH2_OCAL_LSB = 0x15;
	static constexpr uint16_t REG_CH2_GCAL_MSB = 0x16;
	static constexpr uint16_t REG_CH2_GCAL_LSB = 0x17;

	// Registers Channel 3 Specific
	static constexpr uint16_t REG_CH3_CFG      = 0x18;
	static constexpr uint16_t REG_CH3_OCAL_MSB = 0x19;
	static constexpr uint16_t REG_CH3_OCAL_LSB = 0x1A;
	static constexpr uint16_t REG_CH3_GCAL_MSB = 0x1B;
	static constexpr uint16_t REG_CH3_GCAL_LSB = 0x1C;

	// Registers MAP CRC
	static constexpr uint16_t REG_MAP_CRC = 0x3E;

	// ------------------------------------------------------------------------------------

	// Mask READ_REG
	static constexpr uint16_t REGMASK_CMD_READ_REG_ADDRESS = 0x1F80;
	static constexpr uint16_t REGMASK_CMD_READ_REG_BYTES   = 0x007F;

	// Mask Register STATUS
	static constexpr uint16_t REGMASK_STATUS_LOCK     = 0x8000;
	static constexpr uint16_t REGMASK_STATUS_RESYNC   = 0x4000;
	static constexpr uint16_t REGMASK_STATUS_REGMAP   = 0x2000;
	static constexpr uint16_t REGMASK_STATUS_CRC_ERR  = 0x1000;
	static constexpr uint16_t REGMASK_STATUS_CRC_TYPE = 0x0800;
	static constexpr uint16_t REGMASK_STATUS_RESET    = 0x0400;
	static constexpr uint16_t REGMASK_STATUS_WLENGTH  = 0x0300;
	static constexpr uint16_t REGMASK_STATUS_DRDY0    = 0x0001;
	static constexpr uint16_t REGMASK_STATUS_DRDY1    = REGMASK_STATUS_DRDY0 << 1;
	static constexpr uint16_t REGMASK_STATUS_DRDY2    = REGMASK_STATUS_DRDY0 << 2;
	static constexpr uint16_t REGMASK_STATUS_DRDY3    = REGMASK_STATUS_DRDY0 << 3;
	// static constexpr uint16_t REGMASK_STATUS_DRDY4    = REGMASK_STATUS_DRDY0 << 4;
	// static constexpr uint16_t REGMASK_STATUS_DRDY5    = REGMASK_STATUS_DRDY0 << 5;
	// static constexpr uint16_t REGMASK_STATUS_DRDY6    = REGMASK_STATUS_DRDY0 << 6;
	// static constexpr uint16_t REGMASK_STATUS_DRDY7    = REGMASK_STATUS_DRDY0 << 7;

	// Mask Register MODE
	static constexpr uint16_t REGMASK_MODE_REG_CRC_EN = 0x2000;
	static constexpr uint16_t REGMASK_MODE_RX_CRC_EN  = 0x1000;
	static constexpr uint16_t REGMASK_MODE_CRC_TYPE   = 0x0800;
	static constexpr uint16_t REGMASK_MODE_RESET      = 0x0400;
	static constexpr uint16_t REGMASK_MODE_WLENGTH    = 0x0300;
	static constexpr uint16_t REGMASK_MODE_TIMEOUT    = 0x0010;
	static constexpr uint16_t REGMASK_MODE_DRDY_SEL   = 0x000C;
	static constexpr uint16_t REGMASK_MODE_DRDY_HiZ   = 0x0002;
	static constexpr uint16_t REGMASK_MODE_DRDY_FMT   = 0x0001;

	// Mask Register CLOCK
	static constexpr uint16_t REGMASK_CLOCK_CH0_EN = 0x0100;
	static constexpr uint16_t REGMASK_CLOCK_CH1_EN = REGMASK_CLOCK_CH0_EN << 1;
	static constexpr uint16_t REGMASK_CLOCK_CH2_EN = REGMASK_CLOCK_CH0_EN << 2;
	static constexpr uint16_t REGMASK_CLOCK_CH3_EN = REGMASK_CLOCK_CH0_EN << 3;
	// static constexpr uint16_t REGMASK_CLOCK_CH4_EN = REGMASK_CLOCK_CH0_EN << 4;
	// static constexpr uint16_t REGMASK_CLOCK_CH5_EN = REGMASK_CLOCK_CH0_EN << 5;
	// static constexpr uint16_t REGMASK_CLOCK_CH6_EN = REGMASK_CLOCK_CH0_EN << 6;
	// static constexpr uint16_t REGMASK_CLOCK_CH7_EN = REGMASK_CLOCK_CH0_EN << 7;

	static constexpr uint16_t REGMASK_CLOCK_TBM = 0x0020;
	static constexpr uint16_t REGMASK_CLOCK_OSR = 0x001C;
	static constexpr uint16_t REGMASK_CLOCK_PWR = 0x0003;

	// Mask Register GAIN
	static constexpr uint16_t REGMASK_GAIN_PGAGAIN3 = 0x7000;
	static constexpr uint16_t REGMASK_GAIN_PGAGAIN2 = 0x0700;
	static constexpr uint16_t REGMASK_GAIN_PGAGAIN1 = 0x0070;
	static constexpr uint16_t REGMASK_GAIN_PGAGAIN0 = 0x0007;

	// Mask Register CFG
	static constexpr uint16_t REGMASK_CFG_GC_DLY   = 0x1E00;
	static constexpr uint16_t REGMASK_CFG_GC_EN    = 0x0100;
	static constexpr uint16_t REGMASK_CFG_CD_ALLCH = 0x0080;
	static constexpr uint16_t REGMASK_CFG_CD_NUM   = 0x0070;
	static constexpr uint16_t REGMASK_CFG_CD_LEN   = 0x000E;
	static constexpr uint16_t REGMASK_CFG_CD_EN    = 0x0001;

	// Mask Register THRSHLD_LSB
	static constexpr uint16_t REGMASK_THRSHLD_LSB_CD_TH_LSB = 0xFF00;
	static constexpr uint16_t REGMASK_THRSHLD_LSB_DCBLOCK   = 0x000F;

	// Mask Register CHX_CFG
	static constexpr uint16_t REGMASK_CHX_CFG_PHASE       = 0xFFC0;
	static constexpr uint16_t REGMASK_CHX_CFG_DCBLKX_DIS0 = 0x0004;
	static constexpr uint16_t REGMASK_CHX_CFG_MUX         = 0x0003;

	// Mask Register CHX_OCAL_LSB
	static constexpr uint16_t REGMASK_CHX_OCAL0_LSB = 0xFF00;

	// Mask Register CHX_GCAL_LSB
	static constexpr uint16_t REGMASK_CHX_GCAL0_LSB = 0xFF00;

  public:
	static constexpr size_t DATA_WORD_LENGTH = 3; // in bytes

#pragma pack(push, 1)
	struct RawOutput {
		static constexpr size_t SAMPLE_BYTE_ORDER = __ORDER_BIG_ENDIAN__;

		uint16_t status;
		uint8_t unusedStatus;
		uint8_t data[DATA_WORD_LENGTH * NUM_CHANNELS];
		uint16_t crc;
		uint8_t unusedCrc;
	};
#pragma pack(pop)
	static_assert(sizeof(ADS131M0x::RawOutput) == (1 + NUM_CHANNELS + 1) * DATA_WORD_LENGTH,
	              "status, channels, CRC");

	struct HwConfigData {
		uint16_t id;
		uint16_t status;
		uint16_t mode;
		uint16_t clock;
		uint16_t pga;
	};

	void init(gpio_num_t pinCs, gpio_num_t pinDrdy, gpio_num_t pinReset);
	void deinit();
	void setupAccess(spi_host_device_t spiDevice, gpio_num_t clkPin, gpio_num_t misoPin,
	                 gpio_num_t mosiPin);
	void reset();
	bool enableChannel(uint8_t channel, bool enable);
	bool setPowerMode(uint8_t powerMode);
	bool setChannelPGA(uint8_t channel, uint16_t pga);
	bool setChannelInputSelection(uint8_t channel, uint16_t input);
	bool setOsr(uint16_t osr);

	static void interruptHandlerAdcDrdy(void *param);
	void attachISR();
	void setWakeupTask(TaskHandle_t taskToWakeOnDrdy, size_t interval);
	void startAcquisition();
	void stopAcquisition();

	size_t getReadyBatchStartIdx() const { return isrData.tailIndex - isrData.wakeInterval; }
	const RawOutput *rawReadADC(size_t idx) const;

	void stashConfig();
	const HwConfigData *getConfig() const { return &savedConfig; }

	static bool isCrcOk(const RawOutput *data);

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
	spi_transaction_t transDescr;

	gpio_num_t csPin;
	gpio_num_t drdyPin;
	gpio_num_t resetPin;

	HwConfigData savedConfig;

	RawOutput *txSmallBuff;
	RawOutput *rxSmallBuff;

	struct IsrData {
		uint8_t *rxRingBuff;
		lldesc_t *rxDescArray;
		int rxChan;
		size_t headIndex;
		spi_dev_t *spiHw;
		size_t tailIndex;
		size_t wakeInterval;
		TaskHandle_t taskToWake;
	};
	IsrData isrData;
};

#if (CONFIG_MOCK_ADC == 1)

#include <driver/gptimer.h>

class MockAdc {
	gptimer_handle_t gptimer;

  public:
	static constexpr size_t NUM_CHANNELS     = ADS131M0x::NUM_CHANNELS;
	static constexpr size_t DATA_WORD_LENGTH = ADS131M0x::DATA_WORD_LENGTH; // in bytes
	static constexpr size_t DATA_FRAME_SIZE  = ADS131M0x::DATA_FRAME_SIZE;
	typedef ADS131M0x::RawOutput RawOutput;
	typedef ADS131M0x::HwConfigData HwConfigData;

	void init(gpio_num_t pinCs, gpio_num_t pinDrdy, gpio_num_t pinReset) {}
	void deinit() {}
	void setupAccess(spi_host_device_t spiDevice, gpio_num_t clkPin, gpio_num_t misoPin,
	                 gpio_num_t mosiPin) {}
	void setWakeupTask(TaskHandle_t taskToWakeOnDrdy, size_t interval) {
		taskToWake   = taskToWakeOnDrdy;
		wakeInterval = interval;
	}
	void reset() {}
	bool enableChannel(uint8_t channel, bool enable) { return true; }
	bool setChannelPGA(uint8_t channel, uint16_t pga) { return true; }
	bool setPowerMode(uint8_t powerMode) { return true; }
	bool setChannelInputSelection(uint8_t channel, uint16_t input) { return true; }
	bool setOsr(uint16_t osr) { return true; }

	ConfigData savedConfig;
	void stashConfig() {}
	const ConfigData *getConfig() const { return &savedConfig; }

	uint16_t readID() { return 0; }
	uint16_t readSTATUS() { return 0; }
	uint16_t readMODE() { return 0; }
	uint16_t readCLOCK() { return 0; }
	uint16_t readPGA() { return 0; }

	void attachISR();
	void startAcquisition();
	void stopAcquisition();

	size_t getReadyBatchStartIdx() {
		size_t res = tailIndex;
		if (++tailIndex >= wakeInterval) {
			tailIndex = 0;
		}
		return res;
	}

	const RawOutput *rawReadADC(size_t idx) const;
	const RawOutput *rawReadADC() { return rawReadADC(0); };

	static bool isCrcOk(const RawOutput *data) { return true; };

	size_t tailIndex;
	size_t wakeInterval;
	TaskHandle_t taskToWake;
};

typedef MockAdc AdcClass;
#else  // ! (CONFIG_MOCK_ADC == 1)
typedef ADS131M0x AdcClass;
#endif // (CONFIG_MOCK_ADC == 1)

void logADS131M0xConfig(const AdcClass::HwConfigData *cfg);

#endif // ADS131M0x_h
