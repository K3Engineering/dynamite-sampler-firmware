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

struct ADS131M0xIsrData {
	uint8_t *rxRingBuff;
	lldesc_t *rxDescArray;
	spi_dev_t *spiHw;
	int rxChan;
	size_t headIndex;
	size_t tailIndex;
	size_t wakeInterval;
	TaskHandle_t taskToWake;
};

class ADS131M04 {
  public:
	static constexpr size_t NUM_CHANNELS = 4;
	static_assert(NUM_CHANNELS == 4, "Tested on ADS131M04 only");
	static constexpr size_t DATA_WORD_LENGTH = 3; // in bytes

	static constexpr uint16_t RSP_RESET_OK = 0xFF20 + NUM_CHANNELS;

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
	static_assert(sizeof(RawOutput) == (1 + NUM_CHANNELS + 1) * DATA_WORD_LENGTH,
	              "status, channels, CRC");
	static constexpr size_t DMA_PADDED_FRAME_SIZE = (sizeof(RawOutput) + 3) & ~3; // multiple of 4

	void init(gpio_num_t pinCs, gpio_num_t pinDrdy, gpio_num_t pinReset);
	void deinit();
	void setupSpiAccess(spi_host_device_t spiDevice, gpio_num_t clkPin, gpio_num_t misoPin,
	                    gpio_num_t mosiPin);
	void releaseSpi();

	void reset();
	bool setChannelEnable(uint8_t channel, bool enable);
	bool setPowerMode(uint16_t powerMode);
	bool setChannelPGA(uint8_t channel, uint16_t pga);
	bool setChannelInputSelection(uint8_t channel, uint16_t input);
	bool setOsr(uint16_t osr);

	uint16_t readID();
	uint16_t readSTATUS();
	uint16_t readMODE();
	uint16_t readCLOCK();
	uint16_t readPGA();

	void attachISR();
	void setWakeupTask(TaskHandle_t taskToWakeOnDrdy, size_t interval);

	void startAcquisition();
	void stopAcquisition();

	size_t getReadyBatchStartIdx() const { return isrData.tailIndex - isrData.wakeInterval; }
	const RawOutput *rawReadADC(size_t idx) const;

	static bool isCrcOk(const RawOutput *data);

  private:
	uint16_t readRegister(uint8_t address);
	bool writeRegister(uint8_t address, uint16_t value);
	bool writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask);

	spi_device_handle_t spiHandle;
	spi_transaction_t transDescr;

	gpio_num_t csPin;
	gpio_num_t drdyPin;
	gpio_num_t resetPin;

	RawOutput *txSmallBuff;
	RawOutput *rxSmallBuff;

	ADS131M0xIsrData isrData;
};

#if (CONFIG_MOCK_ADC == 1)

#include <driver/gptimer.h>

struct MockAds131xIsrData {
	size_t headIndex;
	size_t tailIndex;
	size_t wakeInterval;
	TaskHandle_t taskToWake;
};

class MockAds131 {
	gptimer_handle_t gptimer;

  public:
	static constexpr size_t NUM_CHANNELS     = ADS131M04::NUM_CHANNELS;
	static constexpr size_t DATA_WORD_LENGTH = ADS131M04::DATA_WORD_LENGTH; // in bytes
	typedef ADS131M04::RawOutput RawOutput;

	void init(gpio_num_t pinCs, gpio_num_t pinDrdy, gpio_num_t pinReset) {}
	void deinit() {}
	void setupSpiAccess(spi_host_device_t spiDevice, gpio_num_t clkPin, gpio_num_t misoPin,
	                    gpio_num_t mosiPin) {}
	void releaseSpi() {}
	void setWakeupTask(TaskHandle_t taskToWakeOnDrdy, size_t interval) {
		isrData.taskToWake   = taskToWakeOnDrdy;
		isrData.wakeInterval = interval;
	}
	void reset() {}
	bool setChannelEnable(uint8_t channel, bool enable) { return true; }
	bool setChannelPGA(uint8_t channel, uint16_t pga) { return true; }
	bool setPowerMode(uint8_t powerMode) { return true; }
	bool setChannelInputSelection(uint8_t channel, uint16_t input) { return true; }
	bool setOsr(uint16_t osr) { return true; }

	uint16_t readID() { return 0; }
	uint16_t readSTATUS() { return 0; }
	uint16_t readMODE() { return 0; }
	uint16_t readCLOCK() { return 0; }
	uint16_t readPGA() { return 0; }

	void attachISR();
	void startAcquisition() {
		isrData.headIndex = isrData.tailIndex = 0;
		gptimer_start(gptimer);
	}
	void stopAcquisition() { gptimer_stop(gptimer); }

	size_t getReadyBatchStartIdx() {
		size_t res = isrData.tailIndex;
		if (++isrData.tailIndex >= isrData.wakeInterval) {
			isrData.tailIndex = 0;
		}
		return res;
	}

	const RawOutput *rawReadADC(size_t idx) const;
	const RawOutput *rawReadADC() { return rawReadADC(0); };

	static bool isCrcOk(const RawOutput *data) { return true; };

	MockAds131xIsrData isrData;
};

typedef MockAds131 AdcClass;

#else // ! (CONFIG_MOCK_ADC == 1)

typedef ADS131M04 AdcClass;

#endif // (CONFIG_MOCK_ADC == 1)

#endif // ADS131M0x_h
