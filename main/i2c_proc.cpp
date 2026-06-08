#include "i2c_proc.h"

#include <freertos/FreeRTOS.h>

#include <driver/i2c_master.h>
#include <esp_log.h>

#include <endian.h>

#include "ADS131M0x_cfg.h"

constexpr char TAG[] = "I2C";

class I2CMasterBus {
	static constexpr gpio_num_t I2C_MASTER_SDA_IO  = GPIO_NUM_46;
	static constexpr gpio_num_t I2C_MASTER_SCL_IO  = GPIO_NUM_3;
	static constexpr i2c_port_num_t I2C_MASTER_NUM = I2C_NUM_0;

	i2c_master_bus_handle_t busHandle;

  public:
	bool setup();
	void release() { i2c_del_master_bus(busHandle); }
	i2c_master_dev_handle_t addDevice(uint16_t addr);
};

class TMP118 {
	static constexpr uint16_t TMP118A_I2C_ADDR = 0x48;
	static constexpr uint16_t TMP118B_I2C_ADDR = 0x49;
	static constexpr uint16_t TMP118C_I2C_ADDR = 0x4A;
	static constexpr uint16_t TMP118D_I2C_ADDR = 0x4B;

	static constexpr uint8_t CONFIG_REG      = 0x01;
	static constexpr uint8_t TEMP_RESULT_REG = 0x00;

	static constexpr uint16_t CFG_RESERVED_DEFAULTS = 0x6030;

	static constexpr uint16_t CFG_CONV_RATE_4S    = 0x0000;
	static constexpr uint16_t CFG_CONV_RATE_1S    = 0x0040;
	static constexpr uint16_t CFG_CONV_RATE_250MS = 0x0080;
	static constexpr uint16_t CFG_CONV_RATE_125MS = 0x00C0;

	static constexpr uint16_t CFG_AVG_NONE      = 0;
	static constexpr uint16_t CFG_AVG_4X_B2B    = 0x01 << 2;
	static constexpr uint16_t CFG_AVG_8X_B2B    = 0x02 << 2;
	static constexpr uint16_t CFG_AVG_MOVING_4X = 0x03 << 2;

	i2c_master_dev_handle_t devHandle;
#pragma pack(push, 1)
	struct ReadCommand {
		uint8_t regPtr;
	};
	struct WriteCommand {
		uint8_t regPtr;
		uint16_t regVal;
	};
#pragma pack(pop)
  public:
	bool config(I2CMasterBus &bus);
	void remove() { i2c_master_bus_rm_device(devHandle); }
	void readTemperature();
};

static inline void delayMSec(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }

bool I2CMasterBus::setup() {
	static constexpr i2c_master_bus_config_t busConfig = {
	    .i2c_port          = I2C_MASTER_NUM,
	    .sda_io_num        = I2C_MASTER_SDA_IO,
	    .scl_io_num        = I2C_MASTER_SCL_IO,
	    .clk_source        = I2C_CLK_SRC_DEFAULT,
	    .glitch_ignore_cnt = 7,
	    .intr_priority     = 0,
	    .trans_queue_depth = 0, // 0 = synchronous (blocking) transactions
	    .flags =
	        {
	            .enable_internal_pullup = true,
	            .allow_pd               = false,
	        },
	};
	esp_err_t res = i2c_new_master_bus(&busConfig, &busHandle);
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "Bus setup failed");
	}
	return ESP_OK == res;
}

i2c_master_dev_handle_t I2CMasterBus::addDevice(uint16_t addr) {
	const i2c_device_config_t devConfig = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address  = addr,
	    .scl_speed_hz    = 100000, // 100 kHz Standard Mode
	    .scl_wait_us     = 0,
	    .flags =
	        {
	            .disable_ack_check = false,
	        },
	};
	i2c_master_dev_handle_t devHandle;
	if (ESP_OK == i2c_master_bus_add_device(busHandle, &devConfig, &devHandle)) {
		return devHandle;
	}
	return nullptr;
}

bool TMP118::config(I2CMasterBus &bus) {
	devHandle = bus.addDevice(TMP118A_I2C_ADDR);
	if (!devHandle) {
		return false;
	}
	const WriteCommand cfg = {
	    .regPtr = CONFIG_REG,
	    .regVal = htobe16(CFG_RESERVED_DEFAULTS | CFG_CONV_RATE_1S | CFG_AVG_8X_B2B),
	};
	esp_err_t cfgErr = i2c_master_transmit(devHandle, (uint8_t *)&cfg, sizeof(cfg), 1000);
	if (cfgErr != ESP_OK) {
		ESP_LOGE(TAG, "config write failed: %d", cfgErr);
		return false;
	}
	return true;
}

volatile float g_latest_i2c_temp = 0.0f;

void TMP118::readTemperature() {
	const ReadCommand cmdGetVal = {
	    .regPtr = TEMP_RESULT_REG,
	};
	uint16_t rxBuff = 0;
	esp_err_t err = i2c_master_transmit_receive(devHandle, (uint8_t *)&cmdGetVal, sizeof(cmdGetVal),
	                                            (uint8_t *)&rxBuff, sizeof(rxBuff), 1000);
	if (err == ESP_OK) {
		int16_t raw = be16toh(rxBuff);
		// Calculate Temperature (16-bit resolution, 0.0078125AC per LSB)
		g_latest_i2c_temp = raw * 0.0078125f;
		ESP_LOGW(TAG, "Temp: %d mC, %.3f C", (raw * 125) / 16, g_latest_i2c_temp);
	} else {
		ESP_LOGE(TAG, "I2C read failed: %d", err);
	}
}

static void taskSetupI2C(void *setupDone) {
	ESP_LOGI(TAG, "setting up I2C on core: %u", esp_cpu_get_core_id());
	I2CMasterBus bus;
	if (!bus.setup()) {
		vTaskDelete(NULL);
	}
	TMP118 sensor;
	if (!sensor.config(bus)) {
		vTaskDelete(NULL);
	}

	*(volatile bool *)setupDone = true;

	while (true) {
		delayMSec(1000);
		sensor.readTemperature();
	}

	vTaskDelete(NULL);
}

void setupI2C(int core) {
	if constexpr (boardConfig.HAS_I2C) {
		volatile bool done = false;
		xTaskCreatePinnedToCore(taskSetupI2C, "task_I2C_setup", 1024 * 2, (void *)&done, 1, NULL,
		                        core);
		while (!done) {
			vTaskDelay(10);
		}
	}
}
