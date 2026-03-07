#include <esp_log.h>

#include "runtime_stats.h"

constexpr char TAG[] = "STAT";

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS

#include <freertos/FreeRTOS.h>

static bool print_one(const TaskStatus_t *prev, const TaskStatus_t *current, uint32_t total) {
	if (prev->xHandle != current->xHandle) {
		return false;
	}
	uint32_t delta_task = current->ulRunTimeCounter - prev->ulRunTimeCounter;
	float percent       = delta_task * 100.0f / total;
	ESP_LOGI(TAG, "%-16s %12u %7.2f%% %8u", current->pcTaskName, delta_task, percent,
	         current->usStackHighWaterMark);
	return true;
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
			ESP_LOGI(TAG, "Task Name            Abs Time   %% Time    Stack");
			for (size_t i = 0; i < current_snapshot_size; i++) {
				for (size_t j = 0; j < prev_snapshot_size; j++) {
					if (print_one(&prev_snapshot[j], &current_snapshot[i], delta_total)) {
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

	ESP_LOGI(TAG, "Free Heap: MALLOC_CAP_DMA %u, MALLOC_CAP_DEFAULT %u",
	         heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
}

static void taskPrintRuntimeStats(void *) {
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(10000));
		print_task_runtime_stats_delta();
	}
	vTaskDelete(NULL);
}

void setupStats(int core) {
	xTaskCreatePinnedToCore(taskPrintRuntimeStats, "task_stats", 1024 * 3, NULL, 0, NULL, core);
}

#else // ! CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS

void setupStats(int) {}

#endif // CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
