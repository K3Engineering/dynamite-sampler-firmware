#include <esp_log.h>
#include <esp_ota_ops.h>

#include "adc_ble_interface.h"
#include "ble_ota_interface.h"
#include "ble_proc.h"

#include "dynamite_uuid.h"

#include "board_cfg.h"

constexpr char TAG[] = "OTA";

#if CONFIG_BOOTLOADER_FACTORY_RESET
static_assert(CONFIG_BOOTLOADER_NUM_PIN_FACTORY_RESET == factoryResetCfg.pin,
              "Menu config != board config");
#if CONFIG_BOOTLOADER_FACTORY_RESET_PIN_HIGH
static_assert(factoryResetCfg.activeLevelHi, "Menu config != board config");
#else
static_assert(!factoryResetCfg.activeLevelHi, "Menu config != board config");
#endif
#else  // CONFIG_BOOTLOADER_FACTORY_RESET
static_assert(!factoryResetCfg.connected(), "Menu config != board config");
#endif // CONFIG_BOOTLOADER_FACTORY_RESET

bool startupDiagnosticIsOk = true;

typedef struct {
	const esp_partition_t *updatePartition;
	esp_ota_handle_t updateHandle;

	size_t numBytesReceived;
	size_t fileSize;
} OtaControlData;

static OtaControlData otaControlData{
    .updatePartition = nullptr,
    .updateHandle = 0,
    .numBytesReceived = 0,
    .fileSize = 0,
};

void *otaControlPtr() { return &otaControlData; }

static bool processOtaBegin(OtaControlData *control, OtaFileSizeType fileSize,
                            OtaReplyType *reply) {
	if (deviceLock != DeviceLock::Open) {
		return false;
	}
	*reply = SVR_CHR_OTA_CONTROL_REQUEST_NAK;
	if (!fileSize) {
		return true;
	}
	control->updatePartition = esp_ota_get_next_update_partition(NULL);
	if ((!control->updatePartition) || (control->updatePartition->size < fileSize)) {
		return true;
	}
	// Erasing the Partition takes some time. Pass in the image size to erase it now.
	// If it erased as the parition is written OTA_WITH_SEQUENTIAL_WRITES,
	// esp_ota_write() can block up to ~70ms, which causes write commands to be dropped.
	esp_err_t err = esp_ota_begin(control->updatePartition, fileSize, &control->updateHandle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_begin error %d (%s)", err, esp_err_to_name(err));
		return true;
	}
	control->fileSize = fileSize;
	control->numBytesReceived = 0;
	*reply = SVR_CHR_OTA_CONTROL_REQUEST_ACK;
	deviceLock = DeviceLock::Ota;
	return true;
}

static bool checkDataSize(const OtaControlData *control) {
	if (control->fileSize == control->numBytesReceived) {
		return true;
	}
	ESP_LOGE(TAG, "Wrong fileSize");
	return false;
}

static bool finishUpdate(esp_ota_handle_t updateHandle) {
	esp_err_t err = esp_ota_end(updateHandle);
	if (err == ESP_OK) {
		return true;
	}
	ESP_LOGE(TAG, "esp_ota_end, err %d", err);
	return false;
}

static bool setBootPartition(const esp_partition_t *updatePartition) {
	esp_err_t err = esp_ota_set_boot_partition(updatePartition);
	if (err == ESP_OK) {
		return true;
	}
	ESP_LOGE(TAG, "esp_ota_set_boot_partition x%X, err %d (%s)!", (size_t)updatePartition->address,
	         err, esp_err_to_name(err));
	return false;
}

static bool processOtaDone(OtaControlData *control, OtaReplyType *reply) {
	if (deviceLock != DeviceLock::Ota) {
		return false;
	}
	ESP_LOGI(TAG, "processOtaDone");
	*reply = SVR_CHR_OTA_CONTROL_DONE_NAK;
	if (!checkDataSize(control)) {
		esp_ota_abort(control->updateHandle);
	} else if (finishUpdate(control->updateHandle) && setBootPartition(control->updatePartition)) {
		*reply = SVR_CHR_OTA_CONTROL_DONE_ACK;
	}
	control->updateHandle = 0;
	control->fileSize = 0;
	control->numBytesReceived = 0;

	deviceLock = DeviceLock::Open;
	return true;
}

static void conditionalRestart(OtaReplyType reply) {
	// restart the ESP to finish the OTA
	static constexpr TickType_t REBOOT_DEEP_SLEEP_TIMEOUT = 500;
	if (SVR_CHR_OTA_CONTROL_DONE_ACK == reply) {
		ESP_LOGI(TAG, "Preparing to restart!");
		vTaskDelay(pdMS_TO_TICKS(REBOOT_DEEP_SLEEP_TIMEOUT));
		esp_restart();
	}
}

void otaOnControlSubscribe() {
	OtaControlData *control = &otaControlData;
	if (deviceLock == DeviceLock::Ota) {
		esp_ota_abort(control->updateHandle);
		deviceLock = DeviceLock::Open;
	}
	if (deviceLock != DeviceLock::Open) {
		return;
	}
	control->updateHandle = 0;
	control->fileSize = 0;
	control->numBytesReceived = 0;
}

int onOtaControlWrite(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt,
                      void *arg) {
	OtaControlData *control = (OtaControlData *)arg;
	const size_t omLen = OS_MBUF_PKTLEN(ctxt->om);
	OtaRequestType code = 0;
	if (omLen >= sizeof(code)) {
		os_mbuf_copydata(ctxt->om, 0, sizeof(code), &code);
	}

	bool res = false;
	OtaReplyType reply = 0;
	switch (code) {
	case SVR_CHR_OTA_CONTROL_REQUEST:
		if (omLen == sizeof(OtaRequestNetworkData)) {
			OtaRequestNetworkData rq;
			os_mbuf_copydata(ctxt->om, 0, sizeof(rq), &rq);
			const OtaFileSizeType fileSize = le32toh(rq.fileSize);
			ESP_LOGI(TAG, "OTA request, size %u", fileSize);
			res = processOtaBegin(control, fileSize, &reply);
		}
		break;
	case SVR_CHR_OTA_CONTROL_DONE:
		if (omLen == sizeof(OtaRequestType)) {
			ESP_LOGI(TAG, "OTA done");
			res = processOtaDone(control, &reply);
		}
		break;
	}

	if (res) {
		// notify the client that it's request has been acknowledged
		if (os_mbuf *om = ble_hs_mbuf_from_flat(&reply, sizeof(reply))) {
			ble_gatts_notify_custom(connHandle, attrHandle, om);
			ESP_LOGI(TAG, "(n)ack %u", reply);
		}
		conditionalRestart(reply);
	}

	return 0;
}

int onOtaDatalWrite(uint16_t connHandle, uint16_t attrHandle, struct ble_gatt_access_ctxt *ctxt,
                    void *arg) {
	if (deviceLock != DeviceLock::Ota) {
		return 0;
	}
	OtaControlData *control = (OtaControlData *)arg;
	const size_t omLen = OS_MBUF_PKTLEN(ctxt->om);
	control->numBytesReceived += omLen;
	if (control->numBytesReceived > control->fileSize) {
		return 0;
	}
	for (os_mbuf *m = ctxt->om; m; m = SLIST_NEXT(m, om_next)) {
		esp_err_t err = esp_ota_write(control->updateHandle, m->om_data, m->om_len);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "esp_ota_write failed (%s)!", esp_err_to_name(err));
		}
	}
	ESP_LOGD(TAG, ".");
	return 0;
}

void otaConditionalRollback() {
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t otaState;
	if (esp_ota_get_state_partition(running, &otaState) == ESP_OK) {
		if (otaState == ESP_OTA_IMG_PENDING_VERIFY) {
			//			if (startupDiagnosticIsOk) {
			ESP_LOGI(TAG, "Diagnostics OK! OTA rollforward...");
			esp_ota_mark_app_valid_cancel_rollback();
			//			} else {
			//				ESP_LOGE(TAG, "Diagnostics failed! OTA rollback...");
			//				esp_ota_mark_app_invalid_rollback_and_reboot();
			//			}
		}
	}
}

void otaOnDisconnect() {
	if (deviceLock != DeviceLock::Ota) {
		return;
	}
	esp_ota_abort(otaControlData.updateHandle);
	otaControlData.updateHandle = 0;
	otaControlData.fileSize = 0;
	otaControlData.numBytesReceived = 0;

	deviceLock = DeviceLock::Open;
	ESP_LOGI(TAG, "otaOnDisconnect");
}
