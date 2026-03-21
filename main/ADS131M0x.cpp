#include "ADS131M0x.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#include <hal/gdma_ll.h>
#include <hal/spi_ll.h>
#include <soc/gdma_channel.h>

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
	txSmallBuff->status            = htobe16(CMD_WRITE_REG | (address << 7));
	*(uint16_t *)txSmallBuff->data = htobe16(value);
	spi_device_polling_transmit(spiHandle, &transDescr);

	txSmallBuff->status = 0;
	spi_device_polling_transmit(spiHandle, &transDescr);

	return be16toh(rxSmallBuff->status) == (RSP_WRITE_REG | (address << 7));
}

uint16_t ADS131M0x::readRegister(uint8_t address) {
	txSmallBuff->status = htobe16(CMD_READ_REG | (address << 7));
	spi_device_polling_transmit(spiHandle, &transDescr);

	txSmallBuff->status = 0;
	spi_device_polling_transmit(spiHandle, &transDescr);

	return be16toh(rxSmallBuff->status);
}

/**
 * @brief Write a value to the register, applying the mask to touch only the necessary bits.
 * It does not carry out the shift of bits (shift), it is necessary to pass the shifted value to the
 * correct position
 */
bool ADS131M0x::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask) {
	// Read the current content of the register
	uint16_t registerContents = readRegister(address);
	// Change the mask bit by bit (it remains 1 in the bits that must not be touched and 0 in the
	// bits to be modified) An AND is performed with the current content of the record. "0" remain
	// in the part to be modified
	registerContents &= ~mask;
	// OR is made with the value to load in the registry. value must be in the correct position
	// (shitf)
	registerContents |= value;
	return writeRegister(address, registerContents);
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

void ADS131M0x::init(gpio_num_t pinCs, gpio_num_t pinDrdy, gpio_num_t pinReset) {
	csPin    = pinCs;
	drdyPin  = pinDrdy;
	resetPin = pinReset;

	txSmallBuff = (RawOutput *)heap_caps_malloc(DMA_PADDED_FRAME_SIZE, MALLOC_CAP_DMA);
	rxSmallBuff = (RawOutput *)heap_caps_malloc(DMA_PADDED_FRAME_SIZE, MALLOC_CAP_DMA);

	isrData.rxRingBuff =
	    (uint8_t *)heap_caps_malloc(DMA_PADDED_FRAME_SIZE * RING_BUFF_SZ, MALLOC_CAP_DMA);
	isrData.rxDescArray =
	    (lldesc_t *)heap_caps_malloc(sizeof(lldesc_t) * RING_BUFF_SZ, MALLOC_CAP_DMA);
	for (size_t i = 0; i < RING_BUFF_SZ; i++) {
		isrData.rxDescArray[i] = {
		    .size   = DMA_PADDED_FRAME_SIZE, // Buffer capacity
		    .length = 0,                     // to be updated by the hw
		    .offset = 0,
		    .sosf   = 0, // Start of sub-frame (unused)
		    .eof    = 1, // End of Frame: DMA stops after this descriptor
		    .owner  = 1, // 1 = Hardware (DMA) owns this
		    .buf    = isrData.rxRingBuff + DMA_PADDED_FRAME_SIZE * i, // Point to data buff
		    .empty  = 0,
		};
	}
	transDescr = {
	    .flags            = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
	    .cmd              = 0,
	    .addr             = 0,
	    .length           = DATA_FRAME_SIZE * 8, // in bits.
	    .rxlength         = DATA_FRAME_SIZE * 8,
	    .override_freq_hz = 0,
	    .user             = nullptr,
	    .tx_buffer        = txSmallBuff,
	    .rx_buffer        = rxSmallBuff,
	};
	gpio_set_level(resetPin, 1);
	gpio_set_level(csPin, 0);

	gpio_set_direction(resetPin, GPIO_MODE_OUTPUT);
	gpio_set_direction(csPin, GPIO_MODE_OUTPUT);
	gpio_set_direction(drdyPin, GPIO_MODE_INPUT);
}

void ADS131M0x::deinit() {
	heap_caps_free(txSmallBuff);
	txSmallBuff = nullptr;
	heap_caps_free(rxSmallBuff);
	rxSmallBuff = nullptr;
	heap_caps_free(isrData.rxRingBuff);
	isrData.rxRingBuff = nullptr;
	heap_caps_free(isrData.rxDescArray);
	isrData.rxDescArray = nullptr;
}

void ADS131M0x::setupAccess(spi_host_device_t spiDevice, gpio_num_t clkPin, gpio_num_t misoPin,
                            gpio_num_t mosiPin) {
	const spi_bus_config_t busCfg = {
	    .mosi_io_num           = mosiPin,
	    .miso_io_num           = misoPin,
	    .sclk_io_num           = clkPin,
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
	esp_err_t ret = spi_bus_initialize(spiDevice, &busCfg, SPI_DMA_CH_AUTO);
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

	spiHandle     = handle;
	isrData.spiHw = SPI_LL_GET_HW(spiDevice);
	assert(isrData.spiHw);
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

const ADS131M0x::RawOutput *IRAM_ATTR ADS131M0x::rawReadADC(size_t idx) const {
	return (RawOutput *)(isrData.rxRingBuff + (idx % RING_BUFF_SZ) * DMA_PADDED_FRAME_SIZE);
}

// When we flag a piece of code with the IRAM_ATTR attribute, the compiled code
// is placed in the ESP32’s Internal RAM (IRAM). Otherwise the code is kept in
// Flash. And Flash on ESP32 is much slower than internal RAM.
void IRAM_ATTR ADS131M0x::interruptHandlerAdcDrdy(void *param) {
	ADS131M0x::IsrData *ctrl = (ADS131M0x::IsrData *)param;

	const size_t idx = ctrl->headIndex++;

	gdma_ll_rx_set_desc_addr(&GDMA, ctrl->rxChan, (uint32_t)&ctrl->rxDescArray[idx % RING_BUFF_SZ]);
	gdma_ll_rx_start(&GDMA, ctrl->rxChan);

	spi_ll_apply_config(ctrl->spiHw);
	spi_ll_user_start(ctrl->spiHw);

	// PIPELINE EXPLANATION:
	// 'idx' is the transfer we are starting via DMA *right now*.
	// Therefore, transfers up to (idx - 1) are guaranteed to have completed in previous ISR runs.
	// If the distance from our last notification exceeds the interval, we wake the task
	// to process the batch of safe, *previously* completed samples while the current one transfers.
	const bool doWake = (idx - ctrl->tailIndex) > ctrl->wakeInterval;
	if (doWake) [[unlikely]] {
		ctrl->tailIndex += ctrl->wakeInterval;
		BaseType_t taskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(ctrl->taskToWake, &taskWoken);
		portYIELD_FROM_ISR(taskWoken);
	}
}

static int findDmaRxChan(spi_dev_t *hw) {
	int gdmaPeriphId = -1;
	if (hw == &GPSPI2) {
		gdmaPeriphId = SOC_GDMA_TRIG_PERIPH_SPI2;
	} else if (hw == &GPSPI3) {
		gdmaPeriphId = SOC_GDMA_TRIG_PERIPH_SPI3;
	} else {
		return -1;
	}
	int chan = -1;
	// Search the available GDMA channels to find which one standard driver allocated to us
	for (size_t i = 0; i < sizeof(GDMA.channel) / sizeof(*GDMA.channel); i++) {
		if (GDMA.channel[i].in.peri_sel.sel == gdmaPeriphId) {
			chan = i;
			break;
		}
	}
	return chan;
}

static inline void clearSpiBuffer(spi_dev_t *hw) {
	for (size_t i = 0; i < sizeof(hw->data_buf) / sizeof(*hw->data_buf); ++i) {
		hw->data_buf[i] = 0;
	}
}

void ADS131M0x::startAcquisition() {
	isrData.headIndex = 0;
	isrData.tailIndex = 0;

	txSmallBuff->status = 0;
	if (ESP_OK != spi_device_polling_start(spiHandle, &transDescr, portMAX_DELAY)) {
		return;
	}
	// spi_device_polling_end() at stopAcquisition()

	// ------- Hijacking DMA/SPI hardware -------
	// Wait for transfer to finish
	while (!spi_ll_usr_is_done(isrData.spiHw))
		;
	isrData.rxChan = findDmaRxChan(isrData.spiHw);
	assert(isrData.rxChan >= 0);

	// TX will read form SPI register buffer, fill it with 0 and disable DMA
	clearSpiBuffer(isrData.spiHw);
	spi_ll_dma_tx_enable(isrData.spiHw, false);

	// RX will use DMA, clear FIFO and enable DMA.
	spi_ll_dma_rx_fifo_reset(isrData.spiHw);
	spi_ll_dma_rx_enable(isrData.spiHw, true);
	// ------- Hijacking done -------

	gpio_set_intr_type(drdyPin, GPIO_INTR_NEGEDGE);
}

void ADS131M0x::stopAcquisition() {
	gpio_set_intr_type(drdyPin, GPIO_INTR_DISABLE);
	spi_device_polling_end(spiHandle, portMAX_DELAY);
}

void ADS131M0x::attachISR() {
	esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
		return;
	}
	gpio_set_intr_type(drdyPin, GPIO_INTR_DISABLE);
	gpio_isr_handler_add(drdyPin, interruptHandlerAdcDrdy, &isrData);
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
                                                  void *userCtx) {
	MockAdc *adc         = (MockAdc *)userCtx;
	BaseType_t taskWoken = pdFALSE;
	static size_t count  = 0;
	if (0 == ++count % adc->wakeInterval) {
		// unblock the task that will read the ADC & handle putting in the buffer
		vTaskNotifyGiveFromISR(adc->taskToWake, &taskWoken);
	}
	return (taskWoken == pdTRUE);
}

void MockAdc::attachISR() {
	static constexpr gptimer_config_t timerConfig = {
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
	gptimer_new_timer(&timerConfig, &gptimer);

	static constexpr gptimer_alarm_config_t alarmConfig = {
	    .alarm_count  = 1000,
	    .reload_count = 0,
	    .flags =
	        {
	            .auto_reload_on_alarm = true,
	        },
	};
	gptimer_set_alarm_action(gptimer, &alarmConfig);

	static constexpr gptimer_event_callbacks_t cbs = {
	    .on_alarm = interruptHandlerMockTimerCb,
	};
	gptimer_register_event_callbacks(gptimer, &cbs, this);
	gptimer_enable(gptimer);
}

void MockAdc::startAcquisition() {
	tailIndex = 0;
	gptimer_start(gptimer);
}

void MockAdc::stopAcquisition() { gptimer_stop(gptimer); }

const MockAdc::RawOutput *IRAM_ATTR MockAdc::rawReadADC(size_t) const {
	static RawOutput a{
	    .status       = htobe16(REGMASK_STATUS_DRDYX),
	    .unusedStatus = 0,
	    .data         = {},
	    .crc          = 0,
	    .unusedCrc    = 0,
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