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

spi_transaction_t ADS131M0x::trans_desc = {
    .flags     = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
    .cmd       = 0,
    .addr      = 0,
    .length    = sizeof(spi2adc) * 8,
    .rxlength  = 0, // equals length
    .user      = nullptr,
    .tx_buffer = &spi2adc,
    .rx_buffer = &adc2spi,
};

bool ADS131M0x::writeRegister(uint8_t address, uint16_t value) {
	spi2adc.status            = htobe16(CMD_WRITE_REG | (address << 7));
	*(uint16_t *)spi2adc.data = htobe16(value);

	spi_device_polling_transmit(spiHandle, &trans_desc);

	spi2adc.status            = 0;
	*(uint16_t *)spi2adc.data = 0;
	spi_device_polling_transmit(spiHandle, &trans_desc);

	return be16toh(adc2spi.status) == (RSP_WRITE_REG | (address << 7));
}

uint16_t ADS131M0x::readRegister(uint8_t address) {
	spi2adc.status = htobe16(CMD_READ_REG | (address << 7));
	spi_device_polling_transmit(spiHandle, &trans_desc);

	spi2adc.status = 0;
	spi_device_polling_transmit(spiHandle, &trans_desc);

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
	register_contents = register_contents & ~mask;
	// OR is made with the value to load in the registry. value must be in the correct position
	// (shitf)
	register_contents = register_contents | value;
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
	spi_bus_config_t buscfg = {
	    .mosi_io_num     = mosi_pin,
	    .miso_io_num     = miso_pin,
	    .sclk_io_num     = clk_pin,
	    .quadwp_io_num   = -1,
	    .quadhd_io_num   = -1,
	    .max_transfer_sz = sizeof(AdcRawOutput),
	};
	esp_err_t ret = spi_bus_initialize(spiDevice, &buscfg, SPI_DMA_CH_AUTO);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi_bus_initialize %d", ret);
		return;
	}

	spi_device_interface_config_t devcfg = {
	    .mode           = 1, // SPI mode 1
	    .clock_speed_hz = spi_clock_speed,
	    .spics_io_num   = -1,
	    .queue_size     = 1, // Queue is not used, but the library requires a non zero value.
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

	if (channel > NUM_CHANNELS_ENABLED)
		return false;
	writeRegisterMasked(REG_GAIN, pga << (channel * 4), REGMASK_GAIN_PGAGAIN0 << (channel * 4));
	return true;
}

bool ADS131M0x::setInputChannelSelection(uint8_t channel, uint8_t input) {
	static_assert(REG_CH1_CFG == REG_CH0_CFG + 5);
	static_assert(REG_CH2_CFG == REG_CH0_CFG + 5 * 2);
	static_assert(REG_CH3_CFG == REG_CH0_CFG + 5 * 3);

	if (channel > NUM_CHANNELS_ENABLED)
		return false;
	writeRegisterMasked(REG_CH0_CFG + channel * 5, input, REGMASK_CHX_CFG_MUX);
	return true;
}

auto ADS131M0x::rawReadADC() -> const AdcRawOutput * {
	spi_device_polling_transmit(spiHandle, &trans_desc);
	return &adc2spi;
}

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

#include <esp_timer.h>

void MockAdc::attachISR(AdcISR isr) {
	esp_timer_handle_t            th;
	const esp_timer_create_args_t tparam{
	    .callback              = isr,
	    .arg                   = nullptr,
	    .dispatch_method       = ESP_TIMER_ISR,
	    .name                  = "MockAdcTmr",
	    .skip_unhandled_events = true,
	};
	esp_timer_create(&tparam, &th);
	esp_timer_start_periodic(th, 1 * 1000);
}

auto MockAdc::rawReadADC() -> const AdcRawOutput * {
	static AdcRawOutput a{
	    .status = 0x1234,
	};
	static uint8_t val = 42;
	for (int i = 3; i < sizeof(a.data); i += 3) {
		a.data[i] = ++val;
	}
	return &a;
}
#endif // CONFIG_MOCK_ADC