#include "board_id.h"

#include <string.h>

#include <esp_log.h>
#include <nvs_flash.h>

#include "user_kvs.h"

constexpr char TAG[] = "BOARD";

static const BoardCfg *gBoardCfg = nullptr;

const BoardCfg *boardCfg() { return gBoardCfg; }

esp_err_t initBoardIdentity() {
	esp_err_t err = nvs_flash_init();
	if (ESP_OK != err) {
		return err;
	}
	if (!initUserKeyValStorage()) {
		return ESP_FAIL;
	}
	char model[BOARD_IDENTITY_NAME_MAX] = {0};
	err = kvsReadBoardModel(model, sizeof(model));
	if (ESP_OK != err) {
		if (ESP_ERR_NVS_NOT_FOUND == err) {
			ESP_LOGW(TAG, "Safe mode: no board identity in flash");
			return ESP_OK;
		}
		if (ESP_ERR_NVS_INVALID_LENGTH == err) {
			ESP_LOGE(TAG, "Safe mode: board identity does not fit %u chars", sizeof(model));
			return ESP_OK;
		}
		return err;
	}
	gBoardCfg = findBoardCfg(model);
	if (gBoardCfg) {
		ESP_LOGI(TAG, "Board: %s", gBoardCfg->name);
	} else {
		ESP_LOGE(TAG, "Safe mode: unknown board identity '%s'", model);
	}
	return ESP_OK;
}
