#ifndef ADS131M0x_h
#define ADS131M0x_h

#include <stddef.h>
#include <stdint.h>

#include <driver/spi_master.h>
#include <soc/gpio_num.h>

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

class ADS131M0x {
	static constexpr size_t NUM_CHANNELS_ENABLED = 4;
	static constexpr size_t DATA_WORD_LENGTH     = 3; // in bytes
	static constexpr size_t ADC_READ_DATA_SIZE =
	    (1 + NUM_CHANNELS_ENABLED + 1) * DATA_WORD_LENGTH; // status, channels, CRC

  public:
#pragma pack(push, 1)
	struct AdcRawOutput {
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

	const AdcRawOutput *rawReadADC();

	uint16_t readID();
	uint16_t readSTATUS();
	uint16_t readMODE();
	uint16_t readCLOCK();
	uint16_t readPGA();

	static bool isCrcOk(const AdcRawOutput *data);

  private:
	uint16_t readRegister(uint8_t address);

	bool writeRegister(uint8_t address, uint16_t value);
	bool writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask);

	spi_device_handle_t spiHandle;

	gpio_num_t csPin;
	gpio_num_t drdyPin;
	gpio_num_t resetPin;

	static AdcRawOutput spi2adc;
	static AdcRawOutput adc2spi;

	static spi_transaction_t transDesc;
};

#if (CONFIG_MOCK_ADC == 1)
class MockAdc {
  public:
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

	uint16_t readID() { return 0; }
	uint16_t readSTATUS() { return 0; }
	uint16_t readMODE() { return 0; }
	uint16_t readCLOCK() { return 0; }
	uint16_t readPGA() { return 0; }

	typedef ADS131M0x::AdcISR       AdcISR;
	typedef ADS131M0x::AdcRawOutput AdcRawOutput;

	void attachISR(AdcISR isr);

	const AdcRawOutput *rawReadADC();

	static bool isCrcOk(const AdcRawOutput *data) { return true; };
};

typedef MockAdc AdcClass;
#else
typedef ADS131M0x AdcClass;
#endif

#endif
