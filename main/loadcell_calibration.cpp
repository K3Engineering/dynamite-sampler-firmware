#include <esp_log.h>
#include <esp_partition.h>

#include "dynamite_sampler_api.h"
#include "loadcell_calibration.h"

constexpr char TAG[] = "CALIBR";

bool readLoadcellCalibration(CalibrationNetworkData *calibration) {
	// This uniquely identifies the partition. This is duplicated in partitions.csv
	static constexpr esp_partition_type_t CALIB_PARTITION_TYPE       = esp_partition_type_t(0x40);
	static constexpr esp_partition_subtype_t CALIB_PARTITION_SUBTYPE = esp_partition_subtype_t(6);
	static constexpr char CALIB_PARTITION_LABEL[]                    = "loadcell_calib";

	const esp_partition_t *ptr = esp_partition_find_first(
	    CALIB_PARTITION_TYPE, CALIB_PARTITION_SUBTYPE, CALIB_PARTITION_LABEL);
	if (!ptr) {
		ESP_LOGE(TAG, "Calibration partition NOT found");
		return false;
	}

	esp_err_t err = esp_partition_read_raw(ptr, 0, calibration->data, sizeof(calibration->data));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Loading calibration data failed, error: %d", err);
		return false;
	}
	return true;
}
