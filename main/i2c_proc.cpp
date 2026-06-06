#include "i2c_proc.h"

#include <freertos/FreeRTOS.h>

#include <driver/i2c_master.h>
#include <esp_log.h>

constexpr char TAG[] = "I2C";

static constexpr gpio_num_t I2C_MASTER_SDA_IO  = GPIO_NUM_46;
static constexpr gpio_num_t I2C_MASTER_SCL_IO  = GPIO_NUM_3;
static constexpr i2c_port_num_t I2C_MASTER_NUM = I2C_NUM_0;
static constexpr uint16_t TMP118_ADDR          = 0x48; // Sensor Address
static constexpr uint8_t CONFIG_REG            = 0x01;
static constexpr uint8_t TEMP_RESULT_REG       = 0x00;

static inline void delayMSec(uint32_t ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }

static void readTemperature(i2c_master_dev_handle_t devHandle) {
	// Read Temperature (Write Register Pointer, then read 2 bytes)
	uint8_t regPtr    = TEMP_RESULT_REG;
	uint8_t rxData[2] = {0};
	esp_err_t err =
	    i2c_master_transmit_receive(devHandle, &regPtr, 1, rxData, sizeof(rxData), 1000);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "I2C read failed: %d", err);
		return;
	}

	// Calculate Temperature (16-bit resolution, 0.0078125°C per LSB)
	int16_t raw = (int16_t)((rxData[0] << 8) | rxData[1]);
	float tempC = raw * 0.0078125f;
	ESP_LOGW(TAG, "Raw: 0x%04X, Temp: %.3f C", (uint16_t)raw, tempC);
}

static void taskSetupI2C(void *setupDone) {
	ESP_LOGI(TAG, "setting up I2C on core: %u", esp_cpu_get_core_id());

	static constexpr i2c_master_bus_config_t busConfig = {
	    .i2c_port          = I2C_MASTER_NUM,
	    .sda_io_num        = I2C_MASTER_SDA_IO,
	    .scl_io_num        = I2C_MASTER_SCL_IO,
	    .clk_source        = I2C_CLK_SRC_DEFAULT,
	    .glitch_ignore_cnt = 7,
	    .intr_priority     = 0,
	    .trans_queue_depth = 8,
	    .flags =
	        {
	            .enable_internal_pullup = true,
	            .allow_pd               = false,
	        },
	};
	i2c_master_bus_handle_t busHandle;
	if (ESP_OK != i2c_new_master_bus(&busConfig, &busHandle)) {
		ESP_LOGE(TAG, "new bus failed");
		vTaskDelete(NULL);
	}

	static constexpr i2c_device_config_t devConfig = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address  = TMP118_ADDR,
	    .scl_speed_hz    = 100000, // 100 kHz Standard Mode
	    .scl_wait_us     = 0,
	    .flags =
	        {
	            .disable_ack_check = false,
	        },
	};
	i2c_master_dev_handle_t devHandle;
	if (ESP_OK != i2c_master_bus_add_device(busHandle, &devConfig, &devHandle)) {
		ESP_LOGE(TAG, "add device failed");
		vTaskDelete(NULL);
	}

	uint8_t writeConfig[3] = {CONFIG_REG, 0x60, 0xB0};
	i2c_master_transmit(devHandle, writeConfig, sizeof(writeConfig), 1000);

	*(volatile bool *)setupDone = true;

	while (true) {
		readTemperature(devHandle);
		delayMSec(5000);
	}

	vTaskDelete(NULL);
}

void setupI2C(int core) {
	volatile bool done = false;
	xTaskCreatePinnedToCore(taskSetupI2C, "task_I2C_setup", 1024 * 2, (void *)&done, 1, NULL, core);
	while (!done)
		vTaskDelay(10);
}
