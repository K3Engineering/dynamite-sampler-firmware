#include "ADS131M0x.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#ifdef USE_LARGE_DMA_BUFF
#include <hal/gdma_ll.h>
#include <hal/spi_ll.h>
#include <soc/gdma_channel.h>
#endif

#include <endian.h>

#include "debug_pin.h"

constexpr char TAG[] = "ADS131";

constexpr size_t DMA_PADDED_FRAME_SIZE = (ADS131M0x::DATA_FRAME_SIZE + 3) & ~3; // multiple of 4

static inline void delayMSec(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }

static constexpr uint16_t crc16ccitt(const void *data, size_t count) {
	static constexpr uint16_t CRC_INIT_VAL = 0xFFFF;
	static constexpr uint16_t CRC_POLYNOM  = 0x1021;

	const uint8_t *ptr = (const uint8_t *)data;
	uint16_t crc       = CRC_INIT_VAL;
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

bool ADS131M0x::writeRegister(uint8_t address, uint16_t value) {
	tx_small_buff->status            = htobe16(CMD_WRITE_REG | (address << 7));
	*(uint16_t *)tx_small_buff->data = htobe16(value);
	spi_device_polling_transmit(spiHandle, &trans_descr);

	tx_small_buff->status = 0;
	spi_device_polling_transmit(spiHandle, &trans_descr);

	return be16toh(rx_small_buff->status) == (RSP_WRITE_REG | (address << 7));
}

uint16_t ADS131M0x::readRegister(uint8_t address) {
	tx_small_buff->status = htobe16(CMD_READ_REG | (address << 7));
	spi_device_polling_transmit(spiHandle, &trans_descr);

	tx_small_buff->status = 0;
	spi_device_polling_transmit(spiHandle, &trans_descr);

	return be16toh(rx_small_buff->status);
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
	delayMSec(10);
}

void ADS131M0x::init(gpio_num_t cs_pin, gpio_num_t drdy_pin, gpio_num_t reset_pin) {
	csPin    = cs_pin;
	drdyPin  = drdy_pin;
	resetPin = reset_pin;

	tx_small_buff = (RawOutput *)heap_caps_malloc(DMA_PADDED_FRAME_SIZE, MALLOC_CAP_DMA);
	rx_small_buff = (RawOutput *)heap_caps_malloc(DMA_PADDED_FRAME_SIZE, MALLOC_CAP_DMA);

#ifdef USE_LARGE_DMA_BUFF
	isr_data.rx_ring_buff =
	    (uint8_t *)heap_caps_malloc(DMA_PADDED_FRAME_SIZE * RING_BUFF_SZ, MALLOC_CAP_DMA);
	isr_data.rx_desc_array =
	    (lldesc_t *)heap_caps_malloc(sizeof(lldesc_t) * RING_BUFF_SZ, MALLOC_CAP_DMA);
	for (size_t i = 0; i < RING_BUFF_SZ; i++) {
		isr_data.rx_desc_array[i] = {
		    .size   = DMA_PADDED_FRAME_SIZE, // Buffer capacity
		    .length = 0,                     // to be updated by the hw
		    .offset = 0,
		    .sosf   = 0, // Start of sub-frame (unused)
		    .eof    = 1, // End of Frame: DMA stops after this descriptor
		    .owner  = 1, // 1 = Hardware (DMA) owns this
		    .buf    = isr_data.rx_ring_buff + DMA_PADDED_FRAME_SIZE * i, // Point to data buff
		    .empty  = 0,
		};
	}
#endif // USE_LARGE_DMA_BUFF
	trans_descr = {
	    .flags            = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
	    .cmd              = 0,
	    .addr             = 0,
	    .length           = DATA_FRAME_SIZE * 8, // in bits.
	    .rxlength         = DATA_FRAME_SIZE * 8,
	    .override_freq_hz = 0,
	    .user             = nullptr,
	    .tx_buffer        = tx_small_buff,
	    .rx_buffer        = rx_small_buff,
	};
	gpio_set_level(resetPin, 1);
	gpio_set_level(csPin, 0);

	gpio_set_direction(resetPin, GPIO_MODE_OUTPUT);
	gpio_set_direction(csPin, GPIO_MODE_OUTPUT);
	gpio_set_direction(drdyPin, GPIO_MODE_INPUT);
}

void ADS131M0x::deinit() {
	heap_caps_free(tx_small_buff);
	tx_small_buff = nullptr;
	heap_caps_free(rx_small_buff);
	rx_small_buff = nullptr;
#ifdef USE_LARGE_DMA_BUFF
	heap_caps_free(isr_data.rx_ring_buff);
	isr_data.rx_ring_buff = nullptr;
	heap_caps_free(isr_data.rx_desc_array);
	isr_data.rx_desc_array = nullptr;
#endif
}

void ADS131M0x::setupAccess(spi_host_device_t spiDevice, gpio_num_t clk_pin, gpio_num_t miso_pin,
                            gpio_num_t mosi_pin) {
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
	    .max_transfer_sz       = DATA_FRAME_SIZE,
	    .flags                 = SPICOMMON_BUSFLAG_MASTER,
	    .isr_cpu_id            = ESP_INTR_CPU_AFFINITY_AUTO,
	    .intr_flags            = 0,
	};
	esp_err_t ret = spi_bus_initialize(spiDevice, &buscfg, SPI_DMA_CH_AUTO);
	assert(ESP_OK == ret);

	const spi_device_interface_config_t devcfg = {
	    .command_bits     = 0,
	    .address_bits     = 0,
	    .dummy_bits       = 0,
	    .mode             = 1, // SPI mode 1
	    .clock_source     = SPI_CLK_SRC_DEFAULT,
	    .duty_cycle_pos   = 0,
	    .cs_ena_pretrans  = 0,
	    .cs_ena_posttrans = 0,
	    .clock_speed_hz   = SPI_MASTER_FREQ_13M,
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
	assert(ESP_OK == ret);
	ret = spi_device_acquire_bus(handle, portMAX_DELAY);
	assert(ESP_OK == ret);

	spiHandle = handle;
#ifdef USE_LARGE_DMA_BUFF
	isr_data.spi_hw = SPI_LL_GET_HW(spiDevice);
	assert(isr_data.spi_hw);
#endif
}

bool ADS131M0x::setPowerMode(uint8_t powerMode) {
	if (powerMode > 3) {
		return false;
	}
	return writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
}

/**
 * @brief set OSR digital filter (see datasheet)
 */
bool ADS131M0x::setOsr(uint16_t osr) {
	if (osr > 7) {
		return false;
	}
	return writeRegisterMasked(REG_CLOCK, osr << 2, REGMASK_CLOCK_OSR);
}

bool ADS131M0x::setChannelPGA(uint8_t channel, uint16_t pga) {
	static_assert(REGMASK_GAIN_PGAGAIN1 == (REGMASK_GAIN_PGAGAIN0 << 4));
	static_assert(REGMASK_GAIN_PGAGAIN2 == (REGMASK_GAIN_PGAGAIN0 << 8));
	static_assert(REGMASK_GAIN_PGAGAIN3 == (REGMASK_GAIN_PGAGAIN0 << 12));

	if (channel >= NUM_CHANNELS_ENABLED) {
		return false;
	}
	return writeRegisterMasked(REG_GAIN, pga << (channel * 4),
	                           REGMASK_GAIN_PGAGAIN0 << (channel * 4));
}

bool ADS131M0x::setPGA(uint8_t pgaChan0, uint8_t pgaChan1, uint8_t pgaChan2, uint8_t pgaChan3) {
	return writeRegister(REG_GAIN, pgaChan0 | (pgaChan1 << 4) | (pgaChan2 << 8) | (pgaChan3 << 12));
}

bool ADS131M0x::setInputChannelSelection(uint8_t channel, uint8_t input) {
	static_assert(REG_CH1_CFG == REG_CH0_CFG + 5);
	static_assert(REG_CH2_CFG == REG_CH0_CFG + 5 * 2);
	static_assert(REG_CH3_CFG == REG_CH0_CFG + 5 * 3);

	if (channel >= NUM_CHANNELS_ENABLED) {
		return false;
	}
	return writeRegisterMasked(REG_CH0_CFG + channel * 5, input, REGMASK_CHX_CFG_MUX);
}

uint16_t ADS131M0x::readID() { return readRegister(REG_ID); }

uint16_t ADS131M0x::readSTATUS() { return readRegister(REG_STATUS); }

uint16_t ADS131M0x::readMODE() { return readRegister(REG_MODE); }

uint16_t ADS131M0x::readCLOCK() { return readRegister(REG_CLOCK); }

uint16_t ADS131M0x::readPGA() { return readRegister(REG_GAIN); }

bool ADS131M0x::isCrcOk(const RawOutput *data) {
	uint16_t crc           = be16toh(data->crc);
	uint16_t calculatedCrc = crc16ccitt(data, sizeof(*data) - DATA_WORD_LENGTH);

	return crc == calculatedCrc;
}

#ifdef USE_LARGE_DMA_BUFF

const ADS131M0x::RawOutput *IRAM_ATTR ADS131M0x::rawReadADC(size_t idx) const {
	return (RawOutput *)(isr_data.rx_ring_buff + (idx % RING_BUFF_SZ) * DMA_PADDED_FRAME_SIZE);
}

// When we flag a piece of code with the IRAM_ATTR attribute, the compiled code
// is placed in the ESP32’s Internal RAM (IRAM). Otherwise the code is kept in
// Flash. And Flash on ESP32 is much slower than internal RAM.
void IRAM_ATTR ADS131M0x::interruptHandlerAdcDrdy(void *param) {
	ADS131M0x::IsrData *ctrl = (ADS131M0x::IsrData *)param;

	const size_t idx = ctrl->head_index++;

	gdma_ll_rx_set_desc_addr(&GDMA, ctrl->rx_chan,
	                         (uint32_t)&ctrl->rx_desc_array[idx % RING_BUFF_SZ]);
	gdma_ll_rx_start(&GDMA, ctrl->rx_chan);

	spi_ll_apply_config(ctrl->spi_hw);
	spi_ll_user_start(ctrl->spi_hw);

	// PIPELINE EXPLANATION:
	// 'idx' is the transfer we are starting via DMA *right now*.
	// Therefore, transfers up to (idx - 1) are guaranteed to have completed in previous ISR runs.
	// If the distance from our last notification exceeds the interval, we wake the task
	// to process the batch of safe, *previously* completed samples while the current one transfers.
	const bool do_wake = (idx - ctrl->tail_index) > ctrl->wake_interval;
	if (do_wake) [[unlikely]] {
		ctrl->tail_index += ctrl->wake_interval;
		BaseType_t taskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(ctrl->task_to_wake, &taskWoken);
		portYIELD_FROM_ISR(taskWoken);
	}
}

static int find_dma_rx_chan(spi_dev_t *hw) {
	int gdma_periph_id = -1;
	if (hw == &GPSPI2) {
		gdma_periph_id = SOC_GDMA_TRIG_PERIPH_SPI2;
	} else if (hw == &GPSPI3) {
		gdma_periph_id = SOC_GDMA_TRIG_PERIPH_SPI3;
	} else {
		return -1;
	}
	int chan = -1;
	// Search the available GDMA channels to find which one standard driver allocated to us
	for (size_t i = 0; i < sizeof(GDMA.channel) / sizeof(*GDMA.channel); i++) {
		if (GDMA.channel[i].in.peri_sel.sel == gdma_periph_id) {
			chan = i;
			break;
		}
	}
	return chan;
}

static inline void clear_spi_buffer(spi_dev_t *hw) {
	for (size_t i = 0; i < sizeof(hw->data_buf) / sizeof(*hw->data_buf); ++i) {
		hw->data_buf[i] = 0;
	}
}

void ADS131M0x::startAcquisition() {
	isr_data.head_index = 0;
	isr_data.tail_index = 0;

	tx_small_buff->status = 0;
	if (ESP_OK != spi_device_polling_start(spiHandle, &trans_descr, portMAX_DELAY)) {
		return;
	}
	// spi_device_polling_end() at stopAcquisition()

	// ------- Hijacking DMA/SPI hardware -------
	// Wait for transfer to finish
	while (!spi_ll_usr_is_done(isr_data.spi_hw))
		;
	isr_data.rx_chan = find_dma_rx_chan(isr_data.spi_hw);
	assert(isr_data.rx_chan >= 0);

	// TX will read form SPI register buffer, fill it with 0 and disable DMA
	clear_spi_buffer(isr_data.spi_hw);
	spi_ll_dma_tx_enable(isr_data.spi_hw, false);

	// RX will use DMA, clear FIFO and enable DMA.
	spi_ll_dma_rx_fifo_reset(isr_data.spi_hw);
	spi_ll_dma_rx_enable(isr_data.spi_hw, true);
	// ------- Hijacking done -------

	gpio_set_intr_type(drdyPin, GPIO_INTR_NEGEDGE);
}

void ADS131M0x::stopAcquisition() {
	gpio_set_intr_type(drdyPin, GPIO_INTR_DISABLE);
	spi_device_polling_end(spiHandle, portMAX_DELAY);
}

#else // ! USE_LARGE_DMA_BUFF

const ADS131M0x::RawOutput *IRAM_ATTR ADS131M0x::rawReadADC() {
	if (ESP_OK != spi_device_polling_transmit(spiHandle, &trans_descr)) {
		ESP_LOGW(TAG, "Reading error");
	} else if ((rx_small_buff->status & htobe16(REGMASK_STATUS_DRDYX)) !=
	           htobe16(REGMASK_STATUS_DRDYX)) {
		ESP_LOGW(TAG, "Reading garbage");
	}
	return rx_small_buff;
}

void IRAM_ATTR ADS131M0x::interruptHandlerAdcDrdy(void *param) {
	ADS131M0x::IsrData *ctrl = (ADS131M0x::IsrData *)param;
	// unblock the task that will read the ADC & handle putting in the buffer
	BaseType_t taskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(ctrl->task_to_wake, &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

void ADS131M0x::startAcquisition() {
	tx_small_buff->status = 0;
	do {
		spi_device_polling_transmit(spiHandle, &trans_descr); // Empty ADC FIFO
	} while (rx_small_buff->status & htobe16(REGMASK_STATUS_DRDYX));
	gpio_set_intr_type(drdyPin, GPIO_INTR_NEGEDGE);
}

void ADS131M0x::stopAcquisition() { gpio_set_intr_type(drdyPin, GPIO_INTR_DISABLE); }

#endif // USE_LARGE_DMA_BUFF

void ADS131M0x::attachISR() {
	esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
		return;
	}
	gpio_set_intr_type(drdyPin, GPIO_INTR_DISABLE);
	gpio_isr_handler_add(drdyPin, interruptHandlerAdcDrdy, &isr_data);
}

// Should not be called while ADC is running
void ADS131M0x::stashConfig() {
	savedConfig = {
	    .id     = readID(),
	    .status = readSTATUS(),
	    .mode   = readMODE(),
	    .clock  = readCLOCK(),
	    .pga    = readPGA(),
	};
}

#if (CONFIG_MOCK_ADC == 1)

#include <driver/gptimer.h>

static bool IRAM_ATTR interruptHandlerMockTimerCb(gptimer_handle_t timer,
                                                  const gptimer_alarm_event_data_t *edata,
                                                  void *user_ctx) {
	MockAdc *adc         = (MockAdc *)user_ctx;
	BaseType_t taskWoken = pdFALSE;
	static size_t count  = 0;
	if (0 == ++count % adc->wake_interval) {
		// unblock the task that will read the ADC & handle putting in the buffer
		vTaskNotifyGiveFromISR(adc->task_to_wake, &taskWoken);
	}
	return (taskWoken == pdTRUE);
}

void MockAdc::attachISR() {
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
	    .on_alarm = interruptHandlerMockTimerCb,
	};
	gptimer_register_event_callbacks(gptimer, &cbs, this);
	gptimer_enable(gptimer);
}

void MockAdc::startAcquisition() {
	tail_index = 0;
	gptimer_start(gptimer);
}

void MockAdc::stopAcquisition() { gptimer_stop(gptimer); }

const MockAdc::RawOutput *IRAM_ATTR MockAdc::rawReadADC(size_t) const {
	static RawOutput a{
	    .status        = htobe16(REGMASK_STATUS_DRDYX),
	    .status_unused = 0,
	    .data          = {},
	    .crc           = 0,
	    .crc_unused    = 0,
	};
	static uint32_t val[NUM_CHANNELS_ENABLED]{0};
	for (size_t i = 0; i < NUM_CHANNELS_ENABLED; ++i) {
		val[i] += i;
		static_assert(AdcClass::DATA_WORD_LENGTH == 3, "Assumes 24-bit ADC sample");
		static_assert(RawOutput::SAMPLE_BYTE_ORDER != __BYTE_ORDER__);
		a.data[i * DATA_WORD_LENGTH]     = val[i] >> 16;
		a.data[i * DATA_WORD_LENGTH + 1] = val[i] >> 8;
		a.data[i * DATA_WORD_LENGTH + 2] = val[i];
	}
	return &a;
}

#endif // CONFIG_MOCK_ADC