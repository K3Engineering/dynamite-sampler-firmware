#include <esp_log.h>
#include <nvs.h>

#include "dynamite_sampler_api.h"
#include "loadcell_calibration.h"

#include <string.h>

constexpr char TAG[] = "CALIBR";

constexpr char CALIBRATION_NSPACE[] = "K3calibration";
static_assert(sizeof(CALIBRATION_NSPACE) <= NVS_NS_NAME_MAX_SIZE);
constexpr size_t CALIBRATION_MAX_KEY_LEN = 15; // does not incude terminating 0
static_assert(CALIBRATION_MAX_KEY_LEN < NVS_KEY_NAME_MAX_SIZE);
constexpr size_t CALIBRATION_MAX_VAL_LEN = 31; // does not incude terminating 0

static size_t splitKeyVal(const char *str) {
	const char *delim = strchr(str, '=');
	if ((delim == nullptr) || (delim == str) || (delim - str > CALIBRATION_MAX_KEY_LEN)) {
		return 0;
	}
	if (strlen(delim + 1) > CALIBRATION_MAX_VAL_LEN) {
		return 0;
	}
	return delim - str;
}

bool writeCalibrationKeyVal(const uint8_t *data, size_t len) {
	char keyVal[CALIBRATION_MAX_KEY_LEN + CALIBRATION_MAX_VAL_LEN + 2];
	if (len >= sizeof(keyVal)) {
		return false;
	}
	memcpy(keyVal, data, len);
	keyVal[len]               = 0;
	const size_t delimiterIdx = splitKeyVal(keyVal);
	if (!delimiterIdx) {
		return false;
	}
	keyVal[delimiterIdx] = 0;
	const char *key      = keyVal;
	const char *val      = keyVal + delimiterIdx + 1;
	if (0 == *val) {
		return false;
	}

	nvs_handle_t handle;
	if (ESP_OK != nvs_open(CALIBRATION_NSPACE, NVS_READWRITE, &handle)) {
		return false;
	}
	esp_err_t err = nvs_set_str(handle, key, val);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	return ESP_OK == err;
}

bool deleteCalibrationKey(const uint8_t *data, size_t len) {
	char key[CALIBRATION_MAX_KEY_LEN];
	if (len >= sizeof(key)) {
		return false;
	}
	memcpy(key, data, len);
	key[len] = 0;

	nvs_handle_t handle;
	if (ESP_OK != nvs_open(CALIBRATION_NSPACE, NVS_READWRITE, &handle)) {
		return false;
	}
	esp_err_t err = nvs_erase_key(handle, key);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	return ESP_OK == err;
}

static size_t compose_one_pair(nvs_handle_t handle, char *dst, const char *key) {
	size_t sz = strlen(key);
	memcpy(dst, key, sz);
	dst[sz++]     = '=';
	size_t length = CALIBRATION_MAX_VAL_LEN + 1;
	if (ESP_OK != nvs_get_str(handle, key, dst + sz, &length)) {
		return 0;
	}
	sz += length - 1;
	dst[sz++] = '\n';
	return sz;
}

bool readCalibrationAll(CalibrationNetworkData *calibration) {
	calibration->data[0] = 0;
	nvs_handle_t handle;
	if (ESP_OK != nvs_open(CALIBRATION_NSPACE, NVS_READONLY, &handle)) {
		return false;
	};
	nvs_iterator_t it;
	esp_err_t err = nvs_entry_find_in_handle(handle, NVS_TYPE_STR, &it);
	if (err != ESP_OK) {
		nvs_close(handle);
		return err == ESP_ERR_NVS_NOT_FOUND;
	}
	size_t recordOffset = 0;
	do {
		nvs_entry_info_t info;
		if (ESP_OK == nvs_entry_info(it, &info)) {
			char record[CALIBRATION_MAX_KEY_LEN + CALIBRATION_MAX_VAL_LEN + 3];
			size_t sz = compose_one_pair(handle, record, info.key);
			if (sz && (recordOffset + sz < sizeof(calibration->data))) {
				memcpy(calibration->data + recordOffset, record, sz);
				recordOffset += sz;
			}
		}
	} while (ESP_OK == nvs_entry_next(&it));
	nvs_release_iterator(it);
	nvs_close(handle);
	calibration->data[recordOffset] = 0;
	return true;
}
