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

constexpr char LOADCELL_NSPACE[] = "loadcell";
static_assert(sizeof(LOADCELL_NSPACE) <= NVS_NS_NAME_MAX_SIZE);
constexpr size_t LOADCELL_MAX_KEY_LEN = 15; // does not incude terminating 0
static_assert(LOADCELL_MAX_KEY_LEN < NVS_KEY_NAME_MAX_SIZE);
constexpr size_t LOADCELL_MAX_VAL_LEN = 31; // does not incude terminating 0

static constexpr size_t splitKeyVal(const char *str) {
	const char *delim = strchr(str, '=');
	if ((delim == nullptr) || (delim == str) || (delim - str > LOADCELL_MAX_KEY_LEN)) {
		return 0;
	}
	if (strlen(delim + 1) > LOADCELL_MAX_VAL_LEN) {
		return 0;
	}
	return delim - str;
}

bool writeLoadcellVal(const uint8_t *data, size_t len) {
	char keyVal[LOADCELL_MAX_KEY_LEN + LOADCELL_MAX_VAL_LEN + 2];
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

	nvs_handle_t handle;
	if (ESP_OK != nvs_open(LOADCELL_NSPACE, NVS_READWRITE, &handle)) {
		return false;
	}
	if (*val) {
		nvs_set_str(handle, key, val);
	} else {
		nvs_erase_key(handle, key);
	}
	nvs_commit(handle);
	nvs_close(handle);
	return true;
}

static size_t compose_one_pair(nvs_handle_t handle, char *dst, const char *key) {
	size_t sz = strlen(key);
	memcpy(dst, key, sz);
	dst[sz++]     = '=';
	size_t length = LOADCELL_MAX_VAL_LEN + 1;
	if (ESP_OK != nvs_get_str(handle, key, dst + sz, &length)) {
		return 0;
	}
	sz += length - 1;
	dst[sz++] = '\n';
	return sz;
}

bool readLoadcellCalibration2(CalibrationNetworkData *calibration) {
	nvs_handle_t handle;
	if (ESP_OK != nvs_open(LOADCELL_NSPACE, NVS_READONLY, &handle)) {
		return false;
	}
	nvs_iterator_t it;
	if (ESP_OK != nvs_entry_find_in_handle(handle, NVS_TYPE_STR, &it)) {
		nvs_close(handle);
		return false;
	}
	size_t recordOffset = 0;
	do {
		nvs_entry_info_t info;
		if (ESP_OK == nvs_entry_info(it, &info)) {
			char record[LOADCELL_MAX_KEY_LEN + LOADCELL_MAX_VAL_LEN + 3];
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
	ESP_LOGI(TAG, "'%s'", calibration->data);
	return true;
}
