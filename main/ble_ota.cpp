#include <esp_log.h>
#include <esp_ota_ops.h>

#include <NimBLEDevice.h>

#include "adc_ble_interface.h"
#include "ble_ota_interface.h"
#include "ble_proc.h"

constexpr char TAG[] = "OTA";

constexpr TickType_t REBOOT_DEEP_SLEEP_TIMEOUT = 500;

constexpr char DEVICE_MANUFACTURER_NAME[] = "3K";
constexpr char DEVICE_MODEL_NUMBER[]      = "0.1d";

//======================== <UUIDs>
constexpr ble_uuid16_t DEVICE_INFO_SVC_UUID =
    BLE_UUID16_INIT(0x180A); // BT Device Information Service
constexpr ble_uuid16_t DEVICE_MAKE_NAME_CHR_UUID =
    BLE_UUID16_INIT(0x2A29); // BT Manufacturer Name String
constexpr ble_uuid16_t DEVICE_MODEL_NUMBER_CHR_UUID =
    BLE_UUID16_INIT(0x2A24); // BT ModelNumberString

constexpr ble_uuid128_t OTA_SVC_UUID = BLE_UUID128_INIT(
    0xd8, 0xe6, 0xfd, 0x1d, 0x4a, 024, 0xc6, 0xb1, 0x53, 0x4c, 0x4c, 0x59, 0x6d, 0xd9, 0xf1, 0xd6);
constexpr ble_uuid128_t OTA_CONTROL_CHR_UUID = BLE_UUID128_INIT(
    0x30, 0xd8, 0xe3, 0x3a, 0x0e, 0x27, 0x22, 0xb7, 0xa4, 0x46, 0xc0, 0x21, 0xaa, 0x71, 0xd6, 0x7a);
constexpr ble_uuid128_t OTA_DATA_CHR_UUID = BLE_UUID128_INIT(
    0xb0, 0xa5, 0xf8, 0x45, 0x8d, 0xca, 0x89, 0x9b, 0xd8, 0x4c, 0x40, 0x1f, 0x88, 0x88, 0x40, 0x23);
//======================== <\UUIDs>

typedef enum {
	SVR_CHR_OTA_CONTROL_NOP,
	SVR_CHR_OTA_CONTROL_REQUEST,
	SVR_CHR_OTA_CONTROL_REQUEST_ACK,
	SVR_CHR_OTA_CONTROL_REQUEST_NAK,
	SVR_CHR_OTA_CONTROL_DONE,
	SVR_CHR_OTA_CONTROL_DONE_ACK,
	SVR_CHR_OTA_CONTROL_DONE_NAK,
} svr_chr_ota_control_val_t;

typedef uint8_t  OtaRequestType;
typedef uint8_t  OtaReplyType;
typedef uint32_t OtaFileSizeType;

typedef struct {
	const esp_partition_t *updatePartition;
	esp_ota_handle_t       updateHandle;

	size_t numBytesReceived;
	size_t fileSize;

	OtaReplyType otaStatus;
	bool         updating;
} OtaControlData;

static OtaControlData otaControlData{
    .updatePartition  = nullptr,
    .updateHandle     = 0,
    .numBytesReceived = 0,
    .fileSize         = 0,
    .otaStatus        = SVR_CHR_OTA_CONTROL_NOP,
    .updating         = false,
};

static bool processOtaBegin(OtaControlData *control) {
	if (control->updating)
		return false;
	if (control->otaStatus != SVR_CHR_OTA_CONTROL_NOP)
		return false;

	control->otaStatus = SVR_CHR_OTA_CONTROL_REQUEST_NAK;
	if (!control->fileSize)
		return true;

	control->updatePartition = esp_ota_get_next_update_partition(NULL);
	if ((!control->updatePartition) || (control->updatePartition->size < control->fileSize))
		return true;

	esp_err_t err =
	    esp_ota_begin(control->updatePartition, control->fileSize, &control->updateHandle);
	if (err == ESP_OK) {
		control->otaStatus = SVR_CHR_OTA_CONTROL_REQUEST_ACK;
		control->updating  = true;
	} else {
		ESP_LOGE(TAG, "esp_ota_begin error %d (%s)", err, esp_err_to_name(err));
	}
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

static bool processOtaDone(OtaControlData *control) {
	if (!control->updating)
		return false;

	ESP_LOGI(TAG, "processOtaDone");
	control->otaStatus = SVR_CHR_OTA_CONTROL_DONE_NAK;
	if (!checkDataSize(control)) {
		esp_ota_abort(control->updateHandle);
	} else if (finishUpdate(control->updateHandle) && setBootPartition(control->updatePartition)) {
		control->otaStatus = SVR_CHR_OTA_CONTROL_DONE_ACK;
	}
	control->updating         = false;
	control->updateHandle     = 0;
	control->fileSize         = 0;
	control->numBytesReceived = 0;
	return true;
}

static void conditionalRestart(const OtaControlData *control) {
	// restart the ESP to finish the OTA
	if (SVR_CHR_OTA_CONTROL_DONE_ACK == control->otaStatus) {
		ESP_LOGI(TAG, "Preparing to restart!");
		vTaskDelay(pdMS_TO_TICKS(REBOOT_DEEP_SLEEP_TIMEOUT));
		esp_restart();
	}
}

static bool processOtaFileSize(OtaControlData *control, OtaFileSizeType netData) {
	if (control->updating)
		return false;

	control->fileSize         = le32toh(netData);
	control->numBytesReceived = 0;
	control->otaStatus        = SVR_CHR_OTA_CONTROL_NOP;
	ESP_LOGI(TAG, "File size: %u", control->fileSize);
	return true;
}

// Implements OTA control flow, while OtaDataChrCallbacks implements binary download.
class OtaControlChrCallbacks : public NimBLECharacteristicCallbacks {
	void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		bool         res   = false;
		const size_t omLen = pCharacteristic->getLength();
		if (sizeof(OtaRequestType) == omLen) {
			OtaRequestType code = pCharacteristic->getValue<OtaRequestType>();
			ESP_LOGI(TAG, "Ctrl recv %u", code);
			if (SVR_CHR_OTA_CONTROL_REQUEST == code) {
				res = processOtaBegin(control);
			} else if (SVR_CHR_OTA_CONTROL_DONE == code) {
				res = processOtaDone(control);
			}
		} else if (sizeof(OtaFileSizeType) == omLen) {
			res = processOtaFileSize(control, pCharacteristic->getValue<OtaFileSizeType>());
		}

		if (res) {
			// notify the client that it's request has been acknowledged
			pCharacteristic->notify(&control->otaStatus, sizeof(control->otaStatus),
			                        connInfo.getConnHandle());
			ESP_LOGI(TAG, "(n)ack %u", control->otaStatus);

			conditionalRestart(control);
		}
	}

	void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
	                 uint16_t subValue) override {
		ESP_LOGI(TAG, "onSubscribe control %u", subValue);
		if (control->updating) {
			esp_ota_abort(control->updateHandle);
		}
		control->updating         = false;
		control->updateHandle     = 0;
		control->fileSize         = 0;
		control->numBytesReceived = 0;
		control->otaStatus        = SVR_CHR_OTA_CONTROL_NOP;
	}

	OtaControlData *const control;

  public:
	explicit OtaControlChrCallbacks(OtaControlData *control_) : control{control_} {}
};

// Hack to expose protected getAttVal()
static inline const void *getChrValuePtr(const NimBLECharacteristic *pCharacteristic) {
	class MyCharacteristic : public NimBLECharacteristic {
	  public:
		const uint8_t *valuePtr() const { return getAttVal().data(); }
	};
	return static_cast<const MyCharacteristic *>(pCharacteristic)->valuePtr();
}

class OtaDataChrCallbacks : public NimBLECharacteristicCallbacks {
	void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		if (!control->updating)
			return;

		const size_t omLen = pCharacteristic->getLength();
		const void  *val   = getChrValuePtr(pCharacteristic);
		control->numBytesReceived += omLen;
		// esp_ota_write can block up to ~70ms. Write commands might be dropped.
		// Use write request (response=True) to make sure nothing is lost.
		// Downside is that it is quite a bit slower.
		if (control->numBytesReceived <= control->fileSize) {
			esp_err_t err = esp_ota_write(control->updateHandle, val, omLen);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "esp_ota_write failed err: %d", err);
			}
		}
		ESP_LOGD(TAG, ".");
	}

	OtaControlData *const control;

  public:
	explicit OtaDataChrCallbacks(OtaControlData *control_) : control{control_} {}
};

void otaConditionalRollback() {
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t   ota_state;
	if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
			bool diagnostic_is_ok = true; // TODO: run diagnostic function
			if (diagnostic_is_ok) {
				ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
				esp_ota_mark_app_valid_cancel_rollback();
			} else {
				ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
				esp_ota_mark_app_invalid_rollback_and_reboot();
			}
		}
	}
}

void setupBleOta(NimBLEServer *server) { // Create the BLE Services
	NimBLEService        *srvDeviceInfo = server->createService(DEVICE_INFO_SVC_UUID.value);
	NimBLECharacteristic *chrDevName    = srvDeviceInfo->createCharacteristic(
        DEVICE_MAKE_NAME_CHR_UUID.value, NIMBLE_PROPERTY::READ, sizeof(DEVICE_MANUFACTURER_NAME));
	chrDevName->setValue(DEVICE_MANUFACTURER_NAME);
	NimBLECharacteristic *chrDevModelNum = srvDeviceInfo->createCharacteristic(
	    DEVICE_MODEL_NUMBER_CHR_UUID.value, NIMBLE_PROPERTY::READ, sizeof(DEVICE_MODEL_NUMBER));
	chrDevModelNum->setValue(DEVICE_MODEL_NUMBER);

	NimBLEService        *srvOTA        = server->createService(&OTA_SVC_UUID);
	NimBLECharacteristic *chrOtaControl = srvOTA->createCharacteristic(
	    &OTA_CONTROL_CHR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY, 16);
	static OtaControlChrCallbacks otaControlCb(&otaControlData);
	chrOtaControl->setCallbacks(&otaControlCb);

	NimBLECharacteristic *chrOtaData =
	    srvOTA->createCharacteristic(&OTA_DATA_CHR_UUID, NIMBLE_PROPERTY::WRITE, 512 + 128);
	static OtaDataChrCallbacks otaDataCb(&otaControlData);
	chrOtaData->setCallbacks(&otaDataCb);

	srvDeviceInfo->start();
	srvOTA->start();
}
