#include <esp_cpu.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_pm.h>

#include "adc_proc.h"
#include "ble_proc.h"

#ifndef GIT_REVISION
#define GIT_REVISION "?"
#endif

constexpr char TAG[] = "DYNA";

// Core0 is for BLE
// Core1 is for everything else. Setup & loop run on core 1.
// ISR is handled on the core that sets it.
constexpr uint32_t CORE_BLE = CONFIG_BT_NIMBLE_PINNED_TO_CORE;
constexpr uint32_t CORE_APP = 1;
static_assert(CORE_BLE != CORE_APP);

extern "C" void app_main(void) {
	ESP_LOGI(TAG, "Git revision: %s", GIT_REVISION);
	ESP_LOGI(TAG, "Running on Core: %u", esp_cpu_get_core_id());

	// ESP_LOGI(TAG, "Config PM SLP IRAM OPT (put lightsleep into ram): %u",
	// CONFIG_PM_SLP_IRAM_OPT);

	uint8_t mac[8]; // size - see esp_efuse_mac_get_default() docs.
	esp_efuse_mac_get_default(mac);
	ESP_LOGI(TAG, "MAC address: %02x%02x%02x%02x%02x%02x", mac[5], mac[4], mac[3], mac[2], mac[1],
	         mac[0]);

	setupBle(CORE_BLE);
	setupAdc(CORE_APP);

	otaConditionalRollback();

	constexpr esp_pm_config_t pmConfig = {
	    .max_freq_mhz       = 80,
	    .min_freq_mhz       = 10,
	    .light_sleep_enable = false,
	};
	if (esp_err_t err = esp_pm_configure(&pmConfig)) {
		ESP_LOGE(TAG, "pm err %d", err);
	}
	// ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
	ESP_LOGI(TAG, "Started!");
}
