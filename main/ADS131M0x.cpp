#include "ADS131M0x.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>

#include <endian.h>

#include "debug_pin.h"

constexpr char TAG[] = "ADS131";

static inline void delayMSec(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }

static constexpr uint16_t crc16ccitt(const void *data, size_t count) {
	static constexpr uint16_t CRC_INIT_VAL = 0xFFFF;
	static constexpr uint16_t CRC_POLYNOM  = 0x1021;

	const uint8_t *ptr = (const uint8_t *)data;
	uint16_t       crc = CRC_INIT_VAL;
	while (count > 0) {
		--count;
		static_assert(sizeof(*ptr << 8) >= sizeof(crc));
		crc ^= *ptr << 8;
		++ptr;
		for (int i = 0; i < 8; ++i) {
			if (crc & 0x8000) {
				crc <<= 1;
				crc ^= CRC_POLYNOM;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

DMA_ATTR ADS131M0x::AdcRawOutput ADS131M0x::spi2adc;
DMA_ATTR ADS131M0x::AdcRawOutput ADS131M0x::adc2spi;

spi_transaction_t ADS131M0x::transDesc = {
    .flags            = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
    .cmd              = 0,
    .addr             = 0,
    .length           = sizeof(spi2adc) * 8, // in bits.
    .rxlength         = 0,                   // 0 makes it rxlength set to the value of .length
    .override_freq_hz = 0,
    .user             = nullptr,
    .tx_buffer        = &spi2adc,
    .rx_buffer        = &adc2spi,
};

bool ADS131M0x::writeRegister(uint8_t address, uint16_t value) {
	spi2adc.status            = htobe16(CMD_WRITE_REG | (address << 7));
	*(uint16_t *)spi2adc.data = htobe16(value);

	spi_device_polling_transmit(spiHandle, &transDesc);

	spi2adc.status            = 0;
	*(uint16_t *)spi2adc.data = 0;
	spi_device_polling_transmit(spiHandle, &transDesc);

	return be16toh(adc2spi.status) == (RSP_WRITE_REG | (address << 7));
}

uint16_t ADS131M0x::readRegister(uint8_t address) {
	spi2adc.status = htobe16(CMD_READ_REG | (address << 7));
	spi_device_polling_transmit(spiHandle, &transDesc);

	spi2adc.status = 0;
	spi_device_polling_transmit(spiHandle, &transDesc);

	return be16toh(adc2spi.status);
}

/**
 * @brief Write a value to the register, applying the mask to touch only the necessary bits.
 * It does not carry out the shift of bits (shift), it is necessary to pass the shifted value to the
 * correct position
 */
bool ADS131M0x::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask) {
	// Read the current content of the register
	uint16_t register_contents = readRegister(address);
	// Change the mask bit by bit (it remains 1 in the bits that must not be touched and 0 in the
	// bits to be modified) An AND is performed with the current content of the record. "0" remain
	// in the part to be modified
	register_contents &= ~mask;
	// OR is made with the value to load in the registry. value must be in the correct position
	// (shitf)
	register_contents |= value;
	return writeRegister(address, register_contents);
}

/// @brief Hardware reset (reset low activ)
void ADS131M0x::reset() {
	gpio_set_level(resetPin, 1);
	delayMSec(100);
	gpio_set_level(resetPin, 0);
	delayMSec(100);
	gpio_set_level(resetPin, 1);
	delayMSec(1);
}

void ADS131M0x::init(gpio_num_t cs_pin, gpio_num_t drdy_pin, gpio_num_t reset_pin) {
	csPin    = cs_pin;
	drdyPin  = drdy_pin;
	resetPin = reset_pin;

	gpio_set_level(resetPin, 1);
	gpio_set_level(csPin, 0);

	gpio_set_direction(resetPin, GPIO_MODE_OUTPUT);
	gpio_set_direction(csPin, GPIO_MODE_OUTPUT);
	gpio_set_direction(drdyPin, GPIO_MODE_INPUT);
}

void ADS131M0x::setupAccess(spi_host_device_t spiDevice, int spi_clock_speed, gpio_num_t clk_pin,
                            gpio_num_t miso_pin, gpio_num_t mosi_pin) {
	const spi_bus_config_t buscfg = {
	    .mosi_io_num           = mosi_pin,
	    .miso_io_num           = miso_pin,
	    .sclk_io_num           = clk_pin,
	    .quadwp_io_num         = -1,
	    .quadhd_io_num         = -1,
	    .data4_io_num          = -1,
	    .data5_io_num          = -1,
	    .data6_io_num          = -1,
	    .data7_io_num          = -1,
	    .data_io_default_level = 0,
	    .max_transfer_sz       = sizeof(AdcRawOutput),
	    .flags                 = SPICOMMON_BUSFLAG_MASTER,
	    .isr_cpu_id            = ESP_INTR_CPU_AFFINITY_AUTO,
	    .intr_flags            = 0,
	};
	esp_err_t ret = spi_bus_initialize(spiDevice, &buscfg, SPI_DMA_CH_AUTO);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi_bus_initialize %d", ret);
		return;
	}

	const spi_device_interface_config_t devcfg = {
	    .command_bits     = 0,
	    .address_bits     = 0,
	    .dummy_bits       = 0,
	    .mode             = 1, // SPI mode 1
	    .clock_source     = SPI_CLK_SRC_DEFAULT,
	    .duty_cycle_pos   = 0,
	    .cs_ena_pretrans  = 0,
	    .cs_ena_posttrans = 0,
	    .clock_speed_hz   = spi_clock_speed,
	    .input_delay_ns   = 0,
	    .sample_point     = SPI_SAMPLING_POINT_PHASE_0,
	    .spics_io_num     = -1,
	    .flags            = 0,
	    .queue_size       = 1, // Queue is not used, but the library requires a non zero value.
	    .pre_cb           = nullptr,
	    .post_cb          = nullptr,
	};
	spi_device_handle_t handle;
	ret = spi_bus_add_device(spiDevice, &devcfg, &handle);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi_bus_add_device %d", ret);
		return;
	}

	ret = spi_device_acquire_bus(handle, portMAX_DELAY);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi_device_acquire_bus %d", ret);
		return;
	}
	spiHandle = handle;
}

bool ADS131M0x::setPowerMode(uint8_t powerMode) {
	if (powerMode > 3) {
		return false;
	}
	writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
	return true;
}

/**
 * @brief set OSR digital filter (see datasheet)
 */
bool ADS131M0x::setOsr(uint16_t osr) {
	if (osr > 7) {
		return false;
	}
	writeRegisterMasked(REG_CLOCK, osr << 2, REGMASK_CLOCK_OSR);
	return true;
}

bool ADS131M0x::setChannelPGA(uint8_t channel, uint16_t pga) {
	static_assert(REGMASK_GAIN_PGAGAIN1 == (REGMASK_GAIN_PGAGAIN0 << 4));
	static_assert(REGMASK_GAIN_PGAGAIN2 == (REGMASK_GAIN_PGAGAIN0 << 8));
	static_assert(REGMASK_GAIN_PGAGAIN3 == (REGMASK_GAIN_PGAGAIN0 << 12));

	if (channel >= NUM_CHANNELS_ENABLED)
		return false;
	writeRegisterMasked(REG_GAIN, pga << (channel * 4), REGMASK_GAIN_PGAGAIN0 << (channel * 4));
	return true;
}

bool ADS131M0x::setPGA(uint8_t pgaChan0, uint8_t pgaChan1, uint8_t pgaChan2, uint8_t pgaChan3) {
	return writeRegister(REG_GAIN, pgaChan0 | (pgaChan1 << 4) | (pgaChan2 << 8) | (pgaChan3 << 12));
}

bool ADS131M0x::setInputChannelSelection(uint8_t channel, uint8_t input) {
	static_assert(REG_CH1_CFG == REG_CH0_CFG + 5);
	static_assert(REG_CH2_CFG == REG_CH0_CFG + 5 * 2);
	static_assert(REG_CH3_CFG == REG_CH0_CFG + 5 * 3);

	if (channel >= NUM_CHANNELS_ENABLED)
		return false;
	writeRegisterMasked(REG_CH0_CFG + channel * 5, input, REGMASK_CHX_CFG_MUX);
	return true;
}

const ADS131M0x::AdcRawOutput *ADS131M0x::rawReadADC() {
	if (ESP_OK == spi_device_polling_start(spiHandle, &transDesc, portMAX_DELAY)) {
		spi_device_polling_end(spiHandle, portMAX_DELAY);
	}
	return &adc2spi;
}

uint16_t ADS131M0x::readID() { return readRegister(REG_ID); }
uint16_t ADS131M0x::readSTATUS() { return readRegister(REG_STATUS); }
uint16_t ADS131M0x::readMODE() { return readRegister(REG_MODE); }
uint16_t ADS131M0x::readCLOCK() { return readRegister(REG_CLOCK); }
uint16_t ADS131M0x::readPGA() { return readRegister(REG_GAIN); }

bool ADS131M0x::isCrcOk(const AdcRawOutput *data) {
	uint16_t crc           = be16toh(data->crc);
	uint16_t calculatedCrc = crc16ccitt(data, sizeof(*data) - DATA_WORD_LENGTH);

	return crc == calculatedCrc;
}

void ADS131M0x::attachISR(AdcISR isr) {
	esp_err_t err = gpio_install_isr_service(0);
	if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
		return;
	gpio_set_intr_type(drdyPin, GPIO_INTR_NEGEDGE);
	gpio_isr_handler_add(drdyPin, isr, nullptr);
}

#if (CONFIG_MOCK_ADC == 1)

#include <driver/gptimer.h>

static bool IRAM_ATTR mockTimerCb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                                  void *user_ctx) {
	bool res = false;
	((MockAdc::AdcISR)user_ctx)(&res);
	return res;
}

void MockAdc::attachISR(AdcISR isr) {
	static constexpr gptimer_config_t timer_config = {
	    .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
	    .direction     = GPTIMER_COUNT_UP,
	    .resolution_hz = 1000 * 1000,
	    .intr_priority = 0,
	    .flags =
	        {
	            .intr_shared         = 0,
	            .allow_pd            = 0,
	            .backup_before_sleep = 0,
	        },
	};
	gptimer_handle_t gptimer = 0;
	gptimer_new_timer(&timer_config, &gptimer);

	static constexpr gptimer_alarm_config_t alarm_config = {
	    .alarm_count  = 1000,
	    .reload_count = 0,
	    .flags =
	        {
	            .auto_reload_on_alarm = true,
	        },
	};
	gptimer_set_alarm_action(gptimer, &alarm_config);

	static constexpr gptimer_event_callbacks_t cbs = {
	    .on_alarm = mockTimerCb,
	};
	gptimer_register_event_callbacks(gptimer, &cbs, (void *)isr);
	gptimer_enable(gptimer);
	gptimer_start(gptimer);
}

const MockAdc::AdcRawOutput *MockAdc::rawReadADC() {
	static AdcRawOutput a{
	    .status        = 0x1234,
	    .status_unused = 0,
	    .data          = {},
	    .crc           = 0,
	    .crc_unused    = 0,
	};
	static uint8_t val = 42;
	for (int i = 3; i < sizeof(a.data); i += 3) {
		a.data[i] = ++val;
	}
	return &a;
}
#endif // CONFIG_MOCK_ADC