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

#include "esp_partition.h"

// awaiting final spec
#pragma pack(push, 1)
struct NvsDataLoadcellCalibration {
	uint32_t calibration0;
	uint32_t calibration1;
	uint32_t calibration2;
};
#pragma pack(pop)

// TODO return esp_err_t instead
static size_t readLoadcellCalibration(void *data, const size_t calibration_length) {

	// This uniquely identifies the partition. This is duplicated in partitions.csv
	constexpr esp_partition_type_t    CALIB_PARTITION_TYPE    = esp_partition_type_t(0x40);
	constexpr esp_partition_subtype_t CALIB_PARTITION_SUBTYPE = esp_partition_subtype_t(6);
	constexpr char                   *CALIB_PARTITION_LABEL   = "loadcell_calib";

	if (const esp_partition_t *ptr = esp_partition_find_first(
	        CALIB_PARTITION_TYPE, CALIB_PARTITION_SUBTYPE, CALIB_PARTITION_LABEL)) {
		if (ESP_OK == esp_partition_read_raw(ptr, 0, data, calibration_length)) {
			ESP_LOGI(TAG, "Read NVS data for loadcell calibration");
			return sizeof(NvsDataLoadcellCalibration);
		}
	}
	return 0;
}

extern "C" void app_main(void) {
	ESP_LOGI(TAG, "Git revision: %s", GIT_REVISION);
	ESP_LOGI(TAG, "Running on Core: %u", esp_cpu_get_core_id());

#ifndef CONFIG_MOCK_ADC
	ESP_LOGI(TAG, "Mock adc flag not set");
#else
	ESP_LOGI(TAG, "Mock ADC flag: %u", CONFIG_MOCK_ADC);
#endif

	// ESP_LOGI(TAG, "Config PM SLP IRAM OPT (put lightsleep into ram): %u",
	// CONFIG_PM_SLP_IRAM_OPT);

	uint64_t _chipmacid = 0LL;
	esp_efuse_mac_get_default((uint8_t *)(&_chipmacid));
	ESP_LOGI(TAG, "MAC address: 0x%" PRIx64, _chipmacid);

	const size_t calibration_length       = sizeof(NvsDataLoadcellCalibration);
	uint8_t      data[calibration_length] = {0};
	readLoadcellCalibration(data, calibration_length);

	// TODO decide what to do if calibration data could not be read
	setupBle(CORE_BLE, data, calibration_length);
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
