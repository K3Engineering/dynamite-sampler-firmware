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
constexpr size_t CALIBRATION_MAX_VAL_LEN = 128; // does not incude terminating 0
static_assert(CALIBRATION_MAX_KEY_LEN + 1 + CALIBRATION_MAX_VAL_LEN + 1 <
              sizeof(CalibrationNetworkData::data));

constexpr size_t splitKeyVal(const char *cmd, size_t cmdLen) {
	for (size_t idx = 0; (idx <= CALIBRATION_MAX_KEY_LEN) && (idx < cmdLen); ++idx) {
		if (cmd[idx] == '=') {
			return idx;
		}
	}
	return 0;
}

bool writeCalibrationKeyVal(const char *cmd, size_t cmdLen) {
	size_t delimiterIdx = splitKeyVal(cmd, cmdLen);
	if ((delimiterIdx == 0) || (cmdLen <= delimiterIdx + 1)) {
		return false;
	}
	char key[CALIBRATION_MAX_KEY_LEN + 1]{0};
	memcpy(key, cmd, delimiterIdx);
	char val[CALIBRATION_MAX_VAL_LEN]{0};
	memcpy(val, cmd + (delimiterIdx + 1), cmdLen - (delimiterIdx + 1));
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
	                                      NVS_READWRITE, &handle)) {
		return false;
	}
	esp_err_t err = nvs_set_str(handle, key, val);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	return ESP_OK == err;
}

bool readCalibrationKey(const char *cmd, size_t cmdLen, char *reply, size_t replySz) {
	if (cmdLen > CALIBRATION_MAX_KEY_LEN) {
		return false;
	}
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
	                                      NVS_READONLY, &handle)) {
		return false;
	}
	char key[CALIBRATION_MAX_KEY_LEN + 1]{0};
	memcpy(key, cmd, cmdLen);
	esp_err_t err = nvs_get_str(handle, key, reply, &replySz);
	nvs_close(handle);
	return ESP_OK == err;
}

bool deleteCalibrationKey(const char *cmd, size_t cmdLen) {
	if (cmdLen > CALIBRATION_MAX_KEY_LEN) {
		return false;
	}
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
	                                      NVS_READWRITE, &handle)) {
		return false;
	}
	char key[CALIBRATION_MAX_KEY_LEN + 1]{0};
	memcpy(key, cmd, cmdLen);
	esp_err_t err = nvs_erase_key(handle, key);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	return ESP_OK == err;
}

bool initCalibrationStorage() { return ESP_OK == nvs_flash_init_partition(CALIBRATION_PARTITION); }

bool readCalibrationN(const char *cmd, size_t cmdLen, char *reply, size_t replySz) {
	const size_t num    = strtoul(cmd, nullptr, 16);
	nvs_handle_t handle = 0;
	if (ESP_OK != nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
	                                      NVS_READONLY, &handle)) {
		return false;
	}
	nvs_iterator_t it = 0;
	esp_err_t err     = nvs_entry_find_in_handle(handle, NVS_TYPE_STR, &it);
	for (size_t i = 0; (ESP_OK == err) && (i < num); ++i) {
		err = nvs_entry_next(&it);
	}
	if (ESP_OK == err) {
		nvs_entry_info_t info;
		err = nvs_entry_info(it, &info);
		if (ESP_OK == err) {
			char val[CALIBRATION_MAX_VAL_LEN + 1];
			size_t valSz = sizeof(val);
			err          = nvs_get_str(handle, info.key, val, &valSz);
			if (ESP_OK == err) {
				strcpy(reply, info.key);
				strcat(reply, "|");
				strcat(reply, val);
			}
		}
	}
	nvs_release_iterator(it);
	nvs_close(handle);
	return ESP_OK == err;
}
/*
bool debugLog() {
    nvs_handle_t handle = 0;
    if (ESP_OK != nvs_open_from_partition(CALIBRATION_PARTITION, DEVICE_CALIBRATION_NSPACE,
                                          NVS_READONLY, &handle)) {
        return false;
    }
    nvs_iterator_t it = 0;
    esp_err_t err     = nvs_entry_find_in_handle(handle, NVS_TYPE_ANY, &it);
    while (ESP_OK == err) {
        nvs_entry_info_t info;
        err = nvs_entry_info(it, &info);
        if (ESP_OK == err) {
            ESP_LOGI(TAG, "'%s::%s' %x", info.namespace_name, info.key, info.type);
            if (info.type == NVS_TYPE_STR) {
                char val[CALIBRATION_MAX_VAL_LEN + 1];
                size_t valSz = sizeof(val);
                if (ESP_OK == nvs_get_str(handle, info.key, val, &valSz)) {
                    ESP_LOGI(TAG, "   ='%s'", val);
                }
            }
        }
        err = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);
    nvs_close(handle);
    return true;
}
*/
