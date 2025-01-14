#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include <NimBLEDevice.h>

#include "adc_ble_interface.h"
#include "ble_ota_interface.h"

#include "ble_proc.h"

#include "esp_ota_ops.h"

#include "debug_pin.h"
#include <HardwareSerial.h>

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
typedef uint16_t OtaPacketSizeType;

typedef struct {
	uint16_t otaControlValHandle;
	uint16_t otaDataValHandle;

	const esp_partition_t *updatePartition;
	esp_ota_handle_t       updateHandle;
	uint16_t               numPkgsReceived;
	OtaPacketSizeType      packetSize;

	OtaReplyType otaStatus;
	bool         updating;
} OtaControlData;

static OtaControlData otaControlData;

static bool processOtaBegin(OtaControlData *control) {
	if (control->updating)
		return false;

	control->updatePartition = esp_ota_get_next_update_partition(NULL);
	esp_err_t err =
	    esp_ota_begin(control->updatePartition, OTA_WITH_SEQUENTIAL_WRITES, &control->updateHandle);
	if (err == ESP_OK) {
		control->otaStatus = SVR_CHR_OTA_CONTROL_REQUEST_ACK;
		control->updating  = true;
	} else {
		esp_ota_abort(control->updateHandle);
		control->otaStatus    = SVR_CHR_OTA_CONTROL_REQUEST_NAK;
		control->updateHandle = 0;
		control->updating     = false;
		// ESP_LOGE(TAG, "esp_ota_begin error %d (%s)", err, esp_err_to_name(err));
		Serial.println("esp_ota_begin error");
	}
	control->numPkgsReceived = 0;

	return true;
}

static bool processOtaDone(OtaControlData *control) {
	if (!control->updating)
		return false;

	Serial.println("processOtaDone");
	control->updating = false;
	esp_err_t err     = esp_ota_end(control->updateHandle);
	Serial.print("esp_ota_end ");
	Serial.println(err);
	if (err == ESP_OK) {
		err = esp_ota_set_boot_partition(control->updatePartition);
		// ESP_LOGE(TAG, "esp_ota_set_boot_partition=%d (%s)!", err,
		//      esp_err_to_name(err))
		Serial.print("esp_ota_set_boot_partition ");
		Serial.println(err);
	}

	control->otaStatus =
	    (err == ESP_OK) ? SVR_CHR_OTA_CONTROL_DONE_ACK : SVR_CHR_OTA_CONTROL_DONE_NAK;

	return true;
}

static void conditionalRestart(OtaControlData *control) {
	// restart the ESP to finish the OTA
	if (SVR_CHR_OTA_CONTROL_DONE_ACK == control->otaStatus) {
		Serial.println("Preparing to restart!");
		vTaskDelay(pdMS_TO_TICKS(REBOOT_DEEP_SLEEP_TIMEOUT));
		esp_restart();
	}
}

// Implements OTA control flow, while OtaDataChrCallbacks implements binary download.
class OtaControlChrCallbacks : public NimBLECharacteristicCallbacks {
	void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		Serial.println("onRead");
		pCharacteristic->setValue(otaControlData.otaStatus);
	}

	void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		const size_t omLen = pCharacteristic->getLength();
		if (sizeof(OtaRequestType) == omLen) {
			auto code = pCharacteristic->getValue<OtaRequestType>();
			Serial.print("OTA ctrl recv ");
			Serial.println(code);
			bool res = false;
			if (SVR_CHR_OTA_CONTROL_REQUEST == code) {
				res = processOtaBegin(&otaControlData);
			} else if (SVR_CHR_OTA_CONTROL_DONE == code) {
				res = processOtaDone(&otaControlData);
			}
			if (res) {
				// notify the client that it's request has been acknowledged
				pCharacteristic->notify(&otaControlData.otaStatus, sizeof(otaControlData.otaStatus),
				                        connInfo.getConnHandle());
				Serial.print("OTA (n)ack ");
				Serial.println(otaControlData.otaStatus);

				conditionalRestart(&otaControlData);
			}
		}
	}

	void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
	                 uint16_t subValue) override {
		Serial.println("onSubscribe");
	}
};

static OtaPacketSizeType decodeOtaPacketSz(OtaPacketSizeType netData) {
	const uint8_t *data = (const uint8_t *)&netData;
	return data[0] + (data[1] << 8);
}

// Hack to expose protected getAttVal()
static inline const void *getChrDataPtr(const NimBLECharacteristic *pCharacteristic) {
	class MyCharacteristic : public NimBLECharacteristic {
	  public:
		const uint8_t *dataBegin() const { return getAttVal().begin(); }
	};
	return static_cast<const MyCharacteristic *>(pCharacteristic)->dataBegin();
}

class OtaDataChrCallbacks : public NimBLECharacteristicCallbacks {
	void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
		const size_t omLen = pCharacteristic->getLength();
		if (otaControlData.updating) {
			const void *val = getChrDataPtr(pCharacteristic);
			// write the data chunk to the partition
			esp_err_t err = esp_ota_write(otaControlData.updateHandle, val, omLen);
			if (err != ESP_OK) {
				Serial.print("esp_ota_write failed err: ");
				Serial.println(err);
			}
			otaControlData.numPkgsReceived++;
		} else {
			if (sizeof(OtaPacketSizeType) == omLen) {
				OtaPacketSizeType val     = pCharacteristic->getValue<OtaPacketSizeType>();
				otaControlData.packetSize = decodeOtaPacketSz(val);
				Serial.print("Packet size: ");
				Serial.println(otaControlData.packetSize);
			}
		}
	}
};

void otaConditionalRollback() {
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t   ota_state;
	if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
			bool diagnostic_is_ok = true; // TODO: run diagnostic function
			if (diagnostic_is_ok) {
				Serial.println("Diagnostics completed successfully! Continuing execution ...");
				esp_ota_mark_app_valid_cancel_rollback();
			} else {
				Serial.println("Diagnostics failed! Start rollback to the previous version ...");
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
	    &OTA_CONTROL_CHR_UUID,
	    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY, 16);
	static OtaControlChrCallbacks otaControlCb;
	chrOtaControl->setCallbacks(&otaControlCb);

	NimBLECharacteristic *chrOtaData =
	    srvOTA->createCharacteristic(&OTA_DATA_CHR_UUID, NIMBLE_PROPERTY::WRITE, 512 + 128);
	static OtaDataChrCallbacks otaDataCb;
	chrOtaData->setCallbacks(&otaDataCb);

	srvDeviceInfo->start();
	srvOTA->start();
}
