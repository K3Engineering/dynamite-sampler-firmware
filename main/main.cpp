#include <esp_cpu.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_pm.h>

#include "adc_proc.h"
#include "ble_proc.h"
#include "runtime_stats.h"

constexpr char TAG[] = "DYNA";

// Core0 is for BLE
// Core1 is for everything else. Setup & loop run on core 1.
// ISR is handled on the core that sets it.
constexpr uint32_t CORE_BLE = CONFIG_BT_NIMBLE_PINNED_TO_CORE;
constexpr uint32_t CORE_APP = 1;
static_assert(CORE_BLE != CORE_APP);

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#include <freertos/FreeRTOS.h>

static void print_one(const TaskStatus_t *prev, const TaskStatus_t *current, uint32_t total) {
	uint32_t delta_task = current->ulRunTimeCounter - prev->ulRunTimeCounter;
	float percent       = delta_task * 100.0f / total;
	ESP_LOGI(TAG, "%-16s  %12u  %7.2f%%", current->pcTaskName, delta_task, percent);
}

static void print_task_runtime_stats_delta() {
	static TaskStatus_t *prev_snapshot = nullptr;
	static size_t prev_snapshot_size   = 0;
	static uint32_t prev_total_runtime = 0;

	size_t current_snapshot_size = uxTaskGetNumberOfTasks();
	auto current_snapshot = (TaskStatus_t *)malloc(current_snapshot_size * sizeof(TaskStatus_t));
	uint32_t current_total_runtime = 0;
	uxTaskGetSystemState(current_snapshot, current_snapshot_size, &current_total_runtime);
	if (prev_snapshot) {
		uint32_t delta_total = current_total_runtime - prev_total_runtime;
		if (delta_total > 0) {
			ESP_LOGI(TAG, "--------------------------------------------------");
			ESP_LOGI(TAG, "Task Name       Abs Time        %% Time");
			for (size_t i = 0; i < current_snapshot_size; i++) {
				for (size_t j = 0; j < prev_snapshot_size; j++) {
					if (prev_snapshot[j].xHandle == current_snapshot[i].xHandle) {
						print_one(&prev_snapshot[j], &current_snapshot[i], delta_total);
						break;
					}
				}
			}
			ESP_LOGI(TAG, "--------------------------------------------------");
		}
	}
	free(prev_snapshot);
	prev_snapshot      = current_snapshot;
	prev_snapshot_size = current_snapshot_size;
	prev_total_runtime = current_total_runtime;
}

static void taskPrintRuntimeStats(void *) {
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(10000));
		print_task_runtime_stats_delta();
	}
	vTaskDelete(NULL);
}

#else

static void taskPrintRuntimeStats(void *) { vTaskDelete(NULL); }

#endif

extern "C" void app_main(void) {
	ESP_LOGI(TAG, "Running on Core: %u", esp_cpu_get_core_id());

	// ESP_LOGI(TAG, "Config PM SLP IRAM OPT (put lightsleep into ram): %u",
	// CONFIG_PM_SLP_IRAM_OPT);

	uint8_t mac[8]; // size - see esp_efuse_mac_get_default() docs.
	esp_efuse_mac_get_default(mac);
	ESP_LOGI(TAG, "MAC address: %02x%02x%02x%02x%02x%02x", mac[5], mac[4], mac[3], mac[2], mac[1],
	         mac[0]);

	setupAdc(CORE_APP);
	setupBle(CORE_BLE);
	setupStats(CORE_BLE);

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
