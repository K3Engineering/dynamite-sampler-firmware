#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include <esp_log.h>
#include <esp_partition.h>

// A large arbirtary size of the partition to read. It will hold all the possible data.
// Its made to be large enough so that the format can change without changing the firmware.
// It can't be larger than the max BLE characteristic length of 512 (0x200)

struct CalibrationData {
	static constexpr size_t CALIB_PARTITION_LENGTH = 0xff;

	uint8_t data[CALIB_PARTITION_LENGTH];
};

static bool readLoadcellCalibration(CalibrationData *calibration) {

	// This uniquely identifies the partition. This is duplicated in partitions.csv
	constexpr esp_partition_type_t    CALIB_PARTITION_TYPE    = esp_partition_type_t(0x40);
	constexpr esp_partition_subtype_t CALIB_PARTITION_SUBTYPE = esp_partition_subtype_t(6);
	constexpr char                    CALIB_PARTITION_LABEL[] = "loadcell_calib";

	const esp_partition_t *ptr = esp_partition_find_first(
	    CALIB_PARTITION_TYPE, CALIB_PARTITION_SUBTYPE, CALIB_PARTITION_LABEL);
	if (!ptr) {
		ESP_LOGE("CALIBR", "Calibration partition NOT found");
		return false;
	}

	esp_err_t err = esp_partition_read_raw(ptr, 0, calibration->data, sizeof(calibration->data));
	if (err != ESP_OK) {
		ESP_LOGE("CALIBR", "Loading calibration data failed, error: %d", err);
		return false;
	}
	return true;
}

#endif // _LOADCELL_CALIBRATION_H
