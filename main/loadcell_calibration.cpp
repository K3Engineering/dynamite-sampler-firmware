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

#include <nvs.h>
#include <string.h>

constexpr char LOADCELL_NSPACE[]      = "loadcell";
constexpr size_t LOADCELL_MAX_VAL_LEN = 32;

bool writeLoadcellStr2(const char *keyVal) {
	nvs_handle_t h;
	if (ESP_OK != nvs_open(LOADCELL_NSPACE, NVS_READWRITE, &h)) {
		return false;
	}
	const char *delim = strchr(keyVal, '=');
	if ((delim == nullptr) || (delim == keyVal)) {
		return false;
	}
	if (delim - keyVal >= NVS_KEY_NAME_MAX_SIZE) {
		return false;
	}
	if (strlen(delim) >= LOADCELL_MAX_VAL_LEN) {
		return false;
	}
	char key[NVS_KEY_NAME_MAX_SIZE] = {0};
	memcpy(key, keyVal, delim - keyVal);
	++delim;
	if (*delim) {
		nvs_set_str(h, key, delim);
	} else {
		nvs_erase_key(h, key);
	}
	nvs_commit(h);
	nvs_close(h);
	return true;
}

bool readLoadcellCalibration2(CalibrationNetworkData *calibration) {
	nvs_handle_t h;
	if (ESP_OK != nvs_open(LOADCELL_NSPACE, NVS_READONLY, &h)) {
		return false;
	}
	nvs_iterator_t it = NULL;
	if (ESP_OK != nvs_entry_find_in_handle(h, NVS_TYPE_STR, &it)) {
		return false;
	}
	*calibration->data = 0;
	do {
		nvs_entry_info_t info;
		nvs_entry_info(it, &info);
		char s[LOADCELL_MAX_VAL_LEN];
		size_t length = sizeof(s);
		nvs_get_str(h, info.key, s, &length);
		strcat((char *)calibration->data, info.key);
		strcat((char *)calibration->data, "=");
		strcat((char *)calibration->data, s);
	} while (ESP_OK == nvs_entry_next(&it));
	nvs_release_iterator(it);
	nvs_close(h);
	ESP_LOGI(TAG, "'%s'", calibration->data);
	return true;
}
