#include <esp_log.h>

#include "runtime_stats.h"

constexpr char TAG[] = "STAT";

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS

#include <freertos/FreeRTOS.h>

static bool printOne(const TaskStatus_t *prev, const TaskStatus_t *current, uint32_t total) {
	if (prev->xHandle != current->xHandle) {
		return false;
	}
	uint32_t deltaTask = current->ulRunTimeCounter - prev->ulRunTimeCounter;
	float percent      = deltaTask * 100.0f / total;
	ESP_LOGI(TAG, "%-16s %12u %7.2f%% %8u", current->pcTaskName, deltaTask, percent,
	         current->usStackHighWaterMark);
	return true;
}

static void printTaskRuntimeStatsDelta() {
	static TaskStatus_t *prevSnapshot = nullptr;
	static size_t prevSnapshotSize    = 0;
	static uint32_t prevTotalRuntime  = 0;

	const size_t currentSnapshotSize = uxTaskGetNumberOfTasks();
	auto currentSnapshot = (TaskStatus_t *)malloc(currentSnapshotSize * sizeof(TaskStatus_t));
	uint32_t currentTotalRuntime = 0;
	uxTaskGetSystemState(currentSnapshot, currentSnapshotSize, &currentTotalRuntime);
	if (prevSnapshot) {
		uint32_t deltaTotal = currentTotalRuntime - prevTotalRuntime;
		if (deltaTotal > 0) {
			ESP_LOGI(TAG, "--------------------------------------------------");
			ESP_LOGI(TAG, "Task Name            Abs Time   %% Time    Stack");
			for (size_t i = 0; i < currentSnapshotSize; i++) {
				for (size_t j = 0; j < prevSnapshotSize; j++) {
					if (printOne(&prevSnapshot[j], &currentSnapshot[i], deltaTotal)) {
						break;
					}
				}
			}
			ESP_LOGI(TAG, "--------------------------------------------------");
		}
	}
	free(prevSnapshot);
	prevSnapshot     = currentSnapshot;
	prevSnapshotSize = currentSnapshotSize;
	prevTotalRuntime = currentTotalRuntime;

	ESP_LOGI(TAG, "Free Heap: DMA %zu, INTERNAL %zu", heap_caps_get_free_size(MALLOC_CAP_DMA),
	         heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

static void taskPrintRuntimeStats(void *) {
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(10000));
		printTaskRuntimeStatsDelta();
	}
	vTaskDelete(NULL);
}

void setupStats(int core) {
	xTaskCreatePinnedToCore(taskPrintRuntimeStats, "task_stats", 1024 * 3, nullptr, 0, nullptr,
	                        core);
}

#else // ! CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS

void setupStats(int) {}

#endif // CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
