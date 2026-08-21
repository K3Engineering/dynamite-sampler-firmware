#include <esp_cpu.h>
#include <esp_log.h>
#include <esp_pm.h>

#include <freertos/FreeRTOS.h>

#include "adc_proc.h"
#include "ble_proc.h"
#include "i2c_proc.h"
#include "runtime_stats.h"

#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#include "tinyusb_default_config.h"

constexpr char TAG[] = "DYNA";

// Core0 is for BLE
// Core1 is for everything else. Setup & loop run on core 1.
// ISR is handled on the core that sets it.
constexpr uint32_t CORE_BLE = CONFIG_BT_NIMBLE_PINNED_TO_CORE;
constexpr uint32_t CORE_APP = 1;
static_assert(CORE_BLE != CORE_APP);

static void printHeader() {
	ESP_LOGI(TAG, "Running on Core: %u", esp_cpu_get_core_id());

	// ESP_LOGI(TAG, "Config PM SLP IRAM OPT (put lightsleep into ram): %u",
	// CONFIG_PM_SLP_IRAM_OPT);
}

static void setPower() {
	constexpr esp_pm_config_t pmConfig = {
	    .max_freq_mhz = 80,
	    .min_freq_mhz = 10,
	    .light_sleep_enable = false,
	};
	if (esp_err_t err = esp_pm_configure(&pmConfig)) {
		ESP_LOGE(TAG, "pm err %d", err);
	}
}

static constexpr tinyusb_cdcacm_itf_t CDC_PORT_DATA = TINYUSB_CDC_ACM_0;
static constexpr tinyusb_cdcacm_itf_t CDC_PORT_FLASH = TINYUSB_CDC_ACM_1;

static void taskHello(void *) {
	while (1) {
		if (tud_cdc_n_connected(CDC_PORT_DATA)) {
			printf("Success! High-Speed CDC stream active under IDF v6.0.\n");
			tinyusb_cdcacm_write_flush(CDC_PORT_DATA, 0);
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}

static void flashPortRxCb(int itf, cdcacm_event_t *event) {
	uint8_t rx_buffer[64];
	size_t rx_size = 0;

	esp_err_t ret =
	    tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, rx_buffer, sizeof(rx_buffer), &rx_size);
	if (ret == ESP_OK && rx_size > 0) {
		if (rx_buffer[0] == 'R') {
			ESP_LOGW(TAG, "Reboot command received on Port 2! Restarting...");
			vTaskDelay(pdMS_TO_TICKS(500)); // Allow buffers to flush
			esp_restart();
		}
	}
}

static void tinyUsb() {
	ESP_LOGI(TAG, "Initializing Decoupled TinyUSB Stack...");

	static constexpr tinyusb_config_t tusbCfg = TINYUSB_DEFAULT_CONFIG();
	ESP_ERROR_CHECK(tinyusb_driver_install(&tusbCfg));

	static constexpr tinyusb_config_cdcacm_t acmCfgData = {
	    .cdc_port = CDC_PORT_DATA,
	    .callback_rx = NULL,
	    .callback_rx_wanted_char = NULL,
	    .callback_line_state_changed = NULL,
	    .callback_line_coding_changed = NULL,
	};
	ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acmCfgData));
	static constexpr tinyusb_config_cdcacm_t acmCfgFlash = {
	    .cdc_port = CDC_PORT_FLASH,
	    .callback_rx = NULL,
	    .callback_rx_wanted_char = NULL,
	    .callback_line_state_changed = NULL,
	    .callback_line_coding_changed = NULL,
	};
	ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acmCfgFlash));

	ESP_ERROR_CHECK(tinyusb_console_init(CDC_PORT_FLASH));
	// esp_log_set_vprintf(vprintf);

	// xTaskCreatePinnedToCore(taskHello, "taskHello", 1024 * 4, NULL, 1, nullptr, 1);

	ESP_LOGI(TAG, "USB-OTG Device Loop Active!");
}

extern "C" void app_main(void) {
	printHeader();

	setupAdc(CORE_APP);
	setupI2C(CORE_APP);
	setupBle(CORE_BLE);
	setupStats(CORE_BLE);

	setPower();

	otaConditionalRollback();

	ESP_LOGI(TAG, "Started! main stack HWM %u", uxTaskGetStackHighWaterMark(NULL));

	tinyUsb();
}
