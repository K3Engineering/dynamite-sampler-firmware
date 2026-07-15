#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "dynamite_sampler_api.h"
#include "loadcell_calibration.h"

#include <stdlib.h>
#include <string.h>

constexpr char TAG[] = "CALIBR";

constexpr char CALIBRATION_PARTITION[] = "DynaPersistent";
static_assert(sizeof(CALIBRATION_PARTITION) <= NVS_PART_NAME_MAX_SIZE);

constexpr char DEVICE_CALIBRATION_NSPACE[] = "Device";
constexpr char SENSOR_CALIBRATION_NSPACE[] = "Extra";
static_assert(sizeof(DEVICE_CALIBRATION_NSPACE) <= NVS_NS_NAME_MAX_SIZE);
static_assert(sizeof(SENSOR_CALIBRATION_NSPACE) <= NVS_NS_NAME_MAX_SIZE);

constexpr size_t CALIBRATION_MAX_KEY_LEN = 15; // does not incude terminating 0
static_assert(CALIBRATION_MAX_KEY_LEN < NVS_KEY_NAME_MAX_SIZE);
constexpr size_t CALIBRATION_MAX_VAL_LEN = 192; // does not incude terminating 0

constexpr size_t CMD_LEN = 4;

constexpr size_t splitKeyVal(const char *str) {
	const char *delim = strchr(str, '=');
	if ((delim == nullptr) || (delim == str) || (delim - str > CALIBRATION_MAX_KEY_LEN)) {
		return 0;
	}
	if (strlen(delim + 1) > CALIBRATION_MAX_VAL_LEN) {
		return 0;
	}
	return delim - str;
}

static_assert(splitKeyVal("012=345") == 3);
static_assert(splitKeyVal("012345") == 0);
static_assert(splitKeyVal("012345678901234=") == 15);
static_assert(splitKeyVal("0123456789012345=") == 0);

bool writeCalibrationKeyVal(CalibrationNetworkData *cmd) {
	char *key                 = (char *)cmd->data + CMD_LEN;
	const size_t delimiterIdx = splitKeyVal(key);
	if (!delimiterIdx) {
		return false;
	}
	const char *val = key + delimiterIdx + 1;
	if (0 == *val) {
		return false;
	}
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
	                                      NVS_READWRITE, &handle)) {
		return false;
	}
	key[delimiterIdx] = 0;
	esp_err_t err     = nvs_set_str(handle, key, val);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	key[delimiterIdx] = '=';
	return ESP_OK == err;
}

bool deleteCalibrationKey(CalibrationNetworkData *cmd) {
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
	                                      NVS_READWRITE, &handle)) {
		return false;
	}
	const char *key = (char *)cmd->data + CMD_LEN;
	esp_err_t err   = nvs_erase_key(handle, key);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	return ESP_OK == err;
}

bool initCalibrationStorage() { return ESP_OK == nvs_flash_init_partition(CALIBRATION_PARTITION); }

bool readCalibrationN(CalibrationNetworkData *cmd) {
	const size_t num    = strtoul((char *)cmd->data + CMD_LEN, nullptr, 16);
	nvs_handle_t handle = 0;
	nvs_iterator_t it   = 0;
	esp_err_t err       = nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
	                                              NVS_READONLY, &handle);
	if (ESP_OK == err) {
		err = nvs_entry_find_in_handle(handle, NVS_TYPE_STR, &it);
		for (size_t i = 0; (ESP_OK == err) && (i < num); ++i) {
			err = nvs_entry_next(&it);
		}
		if (ESP_OK == err) {
			nvs_entry_info_t info;
			err = nvs_entry_info(it, &info);
			if (ESP_OK == err) {
				char val[CALIBRATION_MAX_VAL_LEN];
				size_t valSz = sizeof(val);
				err          = nvs_get_str(handle, info.key, val, &valSz);
				if (ESP_OK == err) {
					strcpy((char *)cmd->data, info.key);
					strcat((char *)cmd->data, "=");
					strcat((char *)cmd->data, val);
				}
			}
		}
	}
	nvs_release_iterator(it);
	nvs_close(handle);
	return ESP_OK == err;
}
