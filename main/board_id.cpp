#include "board_id.h"

#include <string.h>

#include <esp_log.h>
#include <nvs_flash.h>

#include "user_kvs.h"

constexpr char TAG[] = "BOARD";

constexpr char BOARD_MODEL_KEY[] = "board_model";

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
	size_t modelLen = sizeof(model) - 1;
	err = kvsReadFactoryString(BOARD_MODEL_KEY, model, &modelLen);
	if (ESP_ERR_NVS_NOT_FOUND == err) {
		ESP_LOGW(TAG, "Safe mode: no board identity in flash");
		return ESP_OK;
	}
	if (ESP_ERR_NVS_INVALID_LENGTH == err) {
		ESP_LOGE(TAG, "Safe mode: board identity does not fit %u chars", modelLen);
		return ESP_OK;
	}
	if (ESP_OK != err) {
		return err;
	}
	gBoardCfg = findBoardCfg(model);
	if (!gBoardCfg) {
		ESP_LOGE(TAG, "Safe mode: unknown board identity '%s'", model);
		return ESP_OK;
	}
	ESP_LOGI(TAG, "Board: %s", gBoardCfg->name);
	return ESP_OK;
}
