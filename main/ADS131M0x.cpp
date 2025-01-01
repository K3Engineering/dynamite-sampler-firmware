#include "ADS131M0x.h"

#include <SPI.h>
#include <esp32-hal-gpio.h>

#ifdef IS_M02
#define DO_PRAGMA(x) _Pragma(#x)
#define INFO(x)      DO_PRAGMA(message("\nREMARK: " #x))
// INFO Version for ADS131M02
#endif

static constexpr uint16_t crc16ccitt(const uint8_t *ptr, size_t count) {
	static constexpr uint16_t CRC_INIT_VAL = 0xFFFF;
	static constexpr uint16_t CRC_POLYNOM  = 0x1021;

	uint16_t crc = CRC_INIT_VAL;
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

/**
 * @brief Set SPI speed (call bevor "begin" to change default 2MHz)
 *
 * @param cspeed value in Hz
 */
void ADS131M0x::setClockSpeed(uint32_t cspeed) { spiClockSpeed = cspeed; }

/**
 * @brief Write to ADC131M01 or ADC131M04 register
 *
 * @param address
 * @param value
 * @return uint8_t
 */
uint8_t ADS131M0x::writeRegister(uint8_t address, uint16_t value) {
	uint16_t res;
	uint8_t  addressRcv;
	uint8_t  bytesRcv;
	uint16_t cmd = 0;

	digitalWrite(csPin, LOW);
	delayMicroseconds(1);

	cmd = (CMD_WRITE_REG) | (address << 7) | 0;

	// res = spiPort->transfer16(cmd);
	spiPort->transfer16(cmd);
	spiPort->transfer(0x00);

	spiPort->transfer16(value);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

#ifndef IS_M02
	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);
#endif

	res = spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);
#ifndef IS_M02
	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);
#endif
	delayMicroseconds(1);
	digitalWrite(csPin, HIGH);

	addressRcv = (res & REGMASK_CMD_READ_REG_ADDRESS) >> 7;
	bytesRcv   = (res & REGMASK_CMD_READ_REG_BYTES);

	if (addressRcv == address) {
		return bytesRcv + 1;
	}
	return 0;
}

/**
 * @brief
 *
 * @param address
 * @return uint16_t
 */
uint16_t ADS131M0x::readRegister(uint8_t address) {
	uint16_t cmd;
	uint16_t data;

	cmd = CMD_READ_REG | (address << 7 | 0);

	digitalWrite(csPin, LOW);
	delayMicroseconds(1);

	spiPort->transfer16(cmd);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);
#ifndef IS_M02
	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);
#endif
	data = spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);
#ifndef IS_M02
	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);

	spiPort->transfer16(0x0000);
	spiPort->transfer(0x00);
#endif
	delayMicroseconds(1);
	digitalWrite(csPin, HIGH);
	return data;
}

/**
 * @brief Write a value to the register, applying the mask to touch only the necessary bits.
 * It does not carry out the shift of bits (shift), it is necessary to pass the shifted value to the
 * correct position
 *
 * @param address
 * @param value
 * @param mask
 */
void ADS131M0x::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask) {
	// Read the current content of the register
	uint16_t register_contents = readRegister(address);
	// Change the mask bit by bit (it remains 1 in the bits that must not be touched and 0 in the
	// bits to be modified) An AND is performed with the current content of the record. "0" remain
	// in the part to be modified
	register_contents = register_contents & ~mask;
	// OR is made with the value to load in the registry. value must be in the correct position
	// (shitf)
	register_contents = register_contents | value;
	writeRegister(address, register_contents);
}

/// @brief Hardware reset (reset low activ)
/// @param reset_pin
void ADS131M0x::reset(uint8_t reset_pin) {
	pinMode(reset_pin, OUTPUT);
	digitalWrite(reset_pin, HIGH);
	delay(100);
	digitalWrite(reset_pin, LOW);
	delay(100);
	digitalWrite(reset_pin, HIGH);
	delay(1);
}

/**
 * @brief read status cmd-register
 *
 * @return uint16_t
 */
uint16_t ADS131M0x::isResetOK(void) { return (readRegister(CMD_RESET)); }

/**
 * @brief basic initialisation,
 * call '.SetClockSpeed' before to set custom SPI-Clock (default=1MHz),
 * call '.reset' to make extra hardware-reset (optional)
 *
 * @param port      Pointer to SPIClass object
 * @param clk_pin
 * @param miso_pin
 * @param mosi_pin
 * @param cs_pin
 * @param drdy_pin
 */
void ADS131M0x::begin(SPIClass *port, uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin,
                      uint8_t cs_pin, uint8_t drdy_pin) {
	// Set pins up
	csPin   = cs_pin;
	drdyPin = drdy_pin;
	spiPort = port;

	spiPort->begin(clk_pin, miso_pin, mosi_pin, cs_pin); // SCLK, MISO, MOSI, SS
	SPISettings settings(spiClockSpeed, SPI_MSBFIRST, SPI_MODE1);
	spiPort->beginTransaction(settings);
	delay(1);

	pinMode(csPin, OUTPUT);
	digitalWrite(csPin, HIGH); // CS HIGH --> not selected
	pinMode(drdyPin, INPUT);   // DRDY Input
}

/**
 * @brief software test of ADC data is ready
 *
 * @param channel
 * @return int8_t
 */
int8_t ADS131M0x::isDataReadySoft(uint8_t channel) {
	if (channel == 0) {
		return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY0);
	} else if (channel == 1) {
		return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY1);
	}
#ifndef IS_M02
	else if (channel == 2) {
		return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY2);
	} else if (channel == 3) {
		return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY3);
	}
#endif
	return -1;
}

/**
 * @brief read reset status (see datasheet)
 *
 * @return true
 * @return false
 */
bool ADS131M0x::isResetStatus(void) { return (readRegister(REG_STATUS) & REGMASK_STATUS_RESET); }

/**
 * @brief read locked status (see datasheet)
 *
 * @return true
 * @return false
 */
bool ADS131M0x::isLockSPI(void) { return (readRegister(REG_STATUS) & REGMASK_STATUS_LOCK); }

/**
 * @brief set DRDY format (see datasheet)
 *
 * @param drdyFormat
 * @return true
 * @return false
 */
bool ADS131M0x::setDrdyFormat(uint8_t drdyFormat) {
	if (drdyFormat > 1) {
		return false;
	} else {
		writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
		return true;
	}
}

/**
 * @brief set DRDY state (see datasheet)
 *
 * @param drdyState
 * @return true
 * @return false
 */
bool ADS131M0x::setDrdyStateWhenUnavailable(uint8_t drdyState) {
	if (drdyState > 1) {
		return false;
	} else {
		writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
		return true;
	}
}

/**
 * @brief set power mode (see datasheet)
 *
 * @param powerMode
 * @return true
 * @return false
 */
bool ADS131M0x::setPowerMode(uint8_t powerMode) {
	if (powerMode > 3) {
		return false;
	} else {
		writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
		return true;
	}
}

/**
 * @brief set OSR digital filter (see datasheet)
 *
 * @param osr
 * @return true
 * @return false
 */
bool ADS131M0x::setOsr(uint16_t osr) {
	if (osr > 7) {
		return false;
	} else {
		writeRegisterMasked(REG_CLOCK, osr << 2, REGMASK_CLOCK_OSR);
		return true;
	}
}

/**
 * @brief input channel enable
 *
 * @param channel
 * @param enable
 * @return true
 * @return false
 */
bool ADS131M0x::setChannelEnable(uint8_t channel, uint16_t enable) {
	if (channel > 3) {
		return false;
	}
	if (channel == 0) {
		writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
		return true;
	} else if (channel == 1) {
		writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
		return true;
	}
#ifndef IS_M02
	else if (channel == 2) {
		writeRegisterMasked(REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN);
		return true;
	} else if (channel == 3) {
		writeRegisterMasked(REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN);
		return true;
	}
#endif
	return false;
}

/**
 * @brief set gain per channel (see datasheet)
 *
 * @param channel
 * @param pga
 * @return true
 * @return false
 */
bool ADS131M0x::setChannelPGA(uint8_t channel, uint16_t pga) {
	if (channel == 0) {
		writeRegisterMasked(REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0);
		return true;
	} else if (channel == 1) {
		writeRegisterMasked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
		return true;
	}
#ifndef IS_M02
	else if (channel == 2) {
		writeRegisterMasked(REG_GAIN, pga << 8, REGMASK_GAIN_PGAGAIN2);
		return true;
	} else if (channel == 3) {
		writeRegisterMasked(REG_GAIN, pga << 12, REGMASK_GAIN_PGAGAIN3);
		return true;
	}
#endif
	return false;
}

/// @brief Set global Chop (see datasheet)
/// @param global_chop
void ADS131M0x::setGlobalChop(uint16_t global_chop) {
	writeRegisterMasked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}

/// @brief Set global Chop Delay
/// @param delay todo:  ms or us ??
void ADS131M0x::setGlobalChopDelay(uint16_t delay) {
	writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M0x::setInputChannelSelection(uint8_t channel, uint8_t input) {
	if (channel == 0) {
		writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
		return true;
	} else if (channel == 1) {
		writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
		return true;
	}
#ifndef IS_M02
	else if (channel == 2) {
		writeRegisterMasked(REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX);
		return true;
	} else if (channel == 3) {
		writeRegisterMasked(REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX);
		return true;
	}
#endif
	return false;
}

/// @brief set offset calibration per channel
/// @param channel
/// @param offset
/// @return
bool ADS131M0x::setChannelOffsetCalibration(uint8_t channel, int32_t offset) {

	uint16_t MSB = offset >> 8;
	uint8_t  LSB = offset;

	if (channel == 0) {
		writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
		return true;
	} else if (channel == 1) {
		writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
		return true;
	}
#ifndef IS_M02
	else if (channel == 2) {
		writeRegisterMasked(REG_CH2_OCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH2_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
		return true;
	} else if (channel == 3) {
		writeRegisterMasked(REG_CH3_OCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH3_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
		return true;
	}
#endif
	return false;
}

/// @brief set gain calibration per channel
/// @param channel
/// @param gain
/// @return
bool ADS131M0x::setChannelGainCalibration(uint8_t channel, uint32_t gain) {

	uint16_t MSB = gain >> 8;
	uint8_t  LSB = gain;

	if (channel == 0) {
		writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
		return true;
	} else if (channel == 1) {
		writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
		return true;
	}
#ifndef IS_M02
	else if (channel == 2) {
		writeRegisterMasked(REG_CH2_GCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH2_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
		return true;
	} else if (channel == 3) {
		writeRegisterMasked(REG_CH3_GCAL_MSB, MSB, 0xFFFF);
		writeRegisterMasked(REG_CH3_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
		return true;
	}
#endif
	return false;
}

/// @brief hardware-pin test if data is ready
/// @return
bool ADS131M0x::isDataReady() { return LOW == digitalRead(drdyPin); }

static inline int32_t readChannelHelper(const uint8_t *buffer, int index, size_t buffer_length) {
	assert(index >= 0);
	assert(index + 2 < buffer_length);

	int32_t aux =
	    ((buffer[index] << 16) | (buffer[index + 1] << 8) | buffer[index + 2]) & 0x00FFFFFF;
	if (aux > 0x7FFFFF) {
		aux = ((~(aux) & 0x00FFFFFF) + 1) * -1;
	}
	return aux;
}

static inline void optionalDelay() {
#ifndef NO_CS_DELAY
	delayMicroseconds(1);
#endif
}

/// @brief Read ADC port (all Ports)
/// @param
/// @return
ADS131M0x::AdcOutput ADS131M0x::readADC(void) {
	const uint8_t read_length           = ADC_READ_DATA_SIZE;
	const uint8_t txBuffer[read_length] = {0}; // Buffer for SPI transfer
	uint8_t       rxBuffer[read_length] = {};  // Buffer for SPI receive data

	AdcOutput res;

	digitalWrite(csPin, LOW);
	optionalDelay();

	spiPort->transferBytes(txBuffer, rxBuffer, sizeof(rxBuffer));
	// status is bytes 0, 1
	// ch0 is bytes 3, 4, 5
	// ch1 is bytes 6, 7, 8
	// ch2 is bytes 9, 10, 11
	// ch3 is bytes 12, 13, 14
	// CRC is bytes 15, 16

	res.status = (rxBuffer[0] << 8) | rxBuffer[1];
	res.ch0    = readChannelHelper(rxBuffer, 3, sizeof(rxBuffer));
	res.ch1    = readChannelHelper(rxBuffer, 6, sizeof(rxBuffer));
	res.ch2    = readChannelHelper(rxBuffer, 9, sizeof(rxBuffer));
	res.ch3    = readChannelHelper(rxBuffer, 12, sizeof(rxBuffer));

	uint16_t crc            = (rxBuffer[15] << 8) | rxBuffer[16];
	uint16_t calculated_crc = crc16ccitt(rxBuffer, read_length);

	res.crc_match = (crc == calculated_crc);

	optionalDelay();
	digitalWrite(csPin, HIGH);
	return res;
}

ADS131M0x::AdcRawOutput ADS131M0x::rawReadADC() {
	AdcRawOutput  res;
	const uint8_t txBuffer[sizeof(res)] = {0}; // Buffer for SPI duplex transfer

	digitalWrite(csPin, LOW);
	optionalDelay();

	spiPort->transferBytes(txBuffer, reinterpret_cast<uint8_t *>(&res), sizeof(res));

	optionalDelay();
	digitalWrite(csPin, HIGH);
	return res;
}

bool ADS131M0x::isCrcOk(const AdcRawOutput *data) {
	const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&data->crc);

	uint16_t crc           = (ptr[0] << 8) | ptr[1];
	uint16_t calculatedCrc = crc16ccitt(ptr, sizeof(*data) - DATA_WORD_LENGTH);

	return crc == calculatedCrc;
}

bool ADS131M0x::attachISR(AdcISR isr) {
	// TODO: figure out if digitalPinToGPIONumber(drdyPin) is needed
	attachInterrupt(drdyPin, isr, FALLING);
	return true;
}

#include "driver/timer.h"

static bool timerCallback(void *arg) {
	((ADS131M0x::AdcISR)arg)();
	return true;
}

bool MockAdc::attachISR(AdcISR isr) {

	static constexpr timer_config_t config = {
	    .alarm_en    = TIMER_ALARM_EN,
	    .counter_en  = TIMER_PAUSE,
	    .intr_type   = TIMER_INTR_LEVEL,
	    .counter_dir = TIMER_COUNT_UP,
	    .auto_reload = TIMER_AUTORELOAD_EN,
	    .clk_src     = TIMER_SRC_CLK_DEFAULT,
	    .divider     = 0x80,
	};

	constexpr timer_group_t gr  = TIMER_GROUP_0;
	constexpr timer_idx_t   idx = TIMER_0;

	if (ESP_OK != timer_init(gr, idx, &config))
		return false;
	timer_set_counter_value(gr, idx, 0);
	timer_set_alarm_value(gr, idx, 0x4000);
	timer_isr_callback_add(gr, idx, timerCallback, (void *)isr, 0);
	timer_enable_intr(gr, idx);
	timer_start(gr, idx);
	return true;
}

auto MockAdc::rawReadADC() -> AdcRawOutput {
	AdcRawOutput a{
	    .status = 0x1234,
	};
	static uint8_t val;
	for (int i = 3; i < sizeof(a.data); i += 3) {
		a.data[i] = ++val;
	}
	return a;
}