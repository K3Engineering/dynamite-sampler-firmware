#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "dynamite_sampler_api.h"
#include "user_kvs.h"

#include <stdlib.h>
#include <string.h>

constexpr char TAG[] = "KVS";

constexpr char DYNA_PERSIST_PARTITION[] = "DynaPersistent";
static_assert(sizeof(DYNA_PERSIST_PARTITION) <= NVS_PART_NAME_MAX_SIZE);

constexpr char FACTORY_NSPACE[] = "Factory";
constexpr char USER_NSPACE[] = "User";
constexpr char SETTINGS_NSPACE[] = "Settings";
static_assert(sizeof(FACTORY_NSPACE) <= NVS_NS_NAME_MAX_SIZE);
static_assert(sizeof(USER_NSPACE) <= NVS_NS_NAME_MAX_SIZE);
static_assert(sizeof(SETTINGS_NSPACE) <= NVS_NS_NAME_MAX_SIZE);

constexpr size_t USER_KVS_MAX_KEY_LEN = 15; // does not incude terminating 0
static_assert(USER_KVS_MAX_KEY_LEN < NVS_KEY_NAME_MAX_SIZE);
constexpr size_t USER_KVS_MAX_VAL_LEN = 128; // does not incude terminating 0
static_assert(USER_KVS_MAX_KEY_LEN + 1 + USER_KVS_MAX_VAL_LEN + 1 <= USER_KVS_NETWORK_FRAME_LENGTH);

bool initUserKeyValStorage() { return ESP_OK == nvs_flash_init_partition(DYNA_PERSIST_PARTITION); }

constexpr size_t splitKeyVal(const char *cmd) {
	for (size_t idx = 0; (idx <= USER_KVS_MAX_KEY_LEN) && cmd[idx]; ++idx) {
		if (cmd[idx] == '=') {
			size_t valLen = strlen(cmd + idx + 1);
			if (valLen && (valLen <= USER_KVS_MAX_VAL_LEN)) {
				return idx;
			}
			return 0;
		}
	}
	return 0;
}

static bool writeKeyVal(const char *partition, const char *nsp, const char *cmd) {
	// "k...=v..." key=value, null terminated
	size_t delimiterIdx = splitKeyVal(cmd);
	if (delimiterIdx == 0) {
		return false;
	}
	char key[USER_KVS_MAX_KEY_LEN + 1]{0};
	memcpy(key, cmd, delimiterIdx);
	const char *val = cmd + (delimiterIdx + 1);
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(partition, nsp, NVS_READWRITE, &handle)) {
		return false;
	}
	esp_err_t err = nvs_set_str(handle, key, val);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	return ESP_OK == err;
}

static bool readKey(const char *partition, const char *nsp, const char *cmd, char *reply,
                    size_t replySz) {
	// "k..." key, null terminated
	if (strlen(cmd) > USER_KVS_MAX_KEY_LEN) {
		return false;
	}
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(partition, nsp, NVS_READONLY, &handle)) {
		return false;
	}
	esp_err_t err = nvs_get_str(handle, cmd, reply, &replySz);
	nvs_close(handle);
	return ESP_OK == err;
}

static bool deleteKey(const char *partition, const char *nsp, const char *cmd) {
	// "k..." key, null terminated
	if (strlen(cmd) > USER_KVS_MAX_KEY_LEN) {
		return false;
	}
	nvs_handle_t handle;
	if (ESP_OK != nvs_open_from_partition(partition, nsp, NVS_READWRITE, &handle)) {
		return false;
	}
	esp_err_t err = nvs_erase_key(handle, cmd);
	if (ESP_OK == err) {
		err = nvs_commit(handle);
	}
	nvs_close(handle);
	return ESP_OK == err;
}

static bool readByIdx(const char *partition, const char *nsp, const char *cmd, char *reply,
                      size_t replySz) {
	// "N..." number in hex, null terminated
	if (replySz <= USER_KVS_MAX_KEY_LEN + 10) {
		return false;
	}
	const size_t num = strtoul(cmd, nullptr, 16);
	nvs_handle_t handle = 0;
	if (ESP_OK != nvs_open_from_partition(partition, nsp, NVS_READONLY, &handle)) {
		return false;
	}
	nvs_iterator_t it = 0;
	esp_err_t err = nvs_entry_find_in_handle(handle, NVS_TYPE_ANY, &it);
	for (size_t i = 0; (ESP_OK == err) && (i < num); ++i) {
		err = nvs_entry_next(&it);
	}
	if (ESP_OK == err) {
		nvs_entry_info_t info;
		err = nvs_entry_info(it, &info);
		if (ESP_OK == err) {
			size_t keyLen = strlen(info.key);
			memcpy(reply, info.key, keyLen);
			reply[keyLen] = '=';
			utoa(info.type, reply + keyLen + 1, 16);
		}
	}
	nvs_release_iterator(it);
	nvs_close(handle);
	return ESP_OK == err;
}
/*
static bool debugLog() {
    nvs_handle_t handle = 0;
    if (ESP_OK != nvs_open_from_partition(DYNA_PERSIST_PARTITION, FACTORY_NSPACE,
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
                char val[USER_KVS_MAX_VAL_LEN + 1];
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

constexpr const char *nameSpace(char folderCode) {
	switch (folderCode) {
	case UserKvsFolderFactory:
		return FACTORY_NSPACE;
	case UserKvsFolderUser:
		return USER_NSPACE;
	case UserKvsFolderSettings:
		return SETTINGS_NSPACE;
	default:
		return nullptr;
	}
}

constexpr const char *partitionName(char folderCode) {
	switch (folderCode) {
	case UserKvsFolderFactory:
		[[fallthrough]];
	case UserKvsFolderUser:
		return DYNA_PERSIST_PARTITION;
	case UserKvsFolderSettings:
		return NVS_DEFAULT_PART_NAME;
	default:
		return nullptr;
	}
}

bool processKvsCommand(const char *rq, size_t rqLen, char *reply, size_t replySz) {
	const size_t dataOffset = KVS_CMD_LEN + 1;
	if (rqLen < dataOffset) {
		return false;
	}
	const char *part = partitionName(rq[KVS_CMD_LEN]);
	const char *nsp = nameSpace(rq[KVS_CMD_LEN]);
	if (!(part && nsp)) {
		return false;
	}
	if (0 == memcmp(rq, CmdKvsSet, KVS_CMD_LEN)) {
		return writeKeyVal(part, nsp, rq + dataOffset);
	}
	if (0 == memcmp(rq, CmdKvsGet, KVS_CMD_LEN)) {
		return readKey(part, nsp, rq + dataOffset, reply, replySz);
	}
	if (0 == memcmp(rq, CmdKvsDelete, KVS_CMD_LEN)) {
		return deleteKey(part, nsp, rq + dataOffset);
	}
	if (0 == memcmp(rq, CmdKvsGetByIdx, KVS_CMD_LEN)) {
		return readByIdx(part, nsp, rq + dataOffset, reply, replySz);
	}
	return false;
}
