#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include <esp_partition.h>

// This is defined in the partitions.csv
// It can't be larger than the max BLE characteristic length of 512 (0x200)
constexpr size_t CALIB_PARTITION_LENGTH = 0xff;

static esp_err_t readLoadcellCalibration(void *data, const size_t calibration_length) {

	// This uniquely identifies the partition. This is duplicated in partitions.csv
	constexpr esp_partition_type_t    CALIB_PARTITION_TYPE    = esp_partition_type_t(0x40);
	constexpr esp_partition_subtype_t CALIB_PARTITION_SUBTYPE = esp_partition_subtype_t(6);
	constexpr char                   *CALIB_PARTITION_LABEL   = "loadcell_calib";

	if (const esp_partition_t *ptr = esp_partition_find_first(
	        CALIB_PARTITION_TYPE, CALIB_PARTITION_SUBTYPE, CALIB_PARTITION_LABEL)) {
		return esp_partition_read_raw(ptr, 0, data, calibration_length);
	}
	return ESP_FAIL;
}

#endif // _LOADCELL_CALIBRATION_H
