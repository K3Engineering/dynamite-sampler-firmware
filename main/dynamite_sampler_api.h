#ifndef __dynamite_sampler_api_h__
#define __dynamite_sampler_api_h__

#include <stdint.h>

#include <host/ble_uuid.h>

//======================== <UUIDs>

constexpr ble_uuid16_t DEVICE_INFO_SVC_UUID16         = BLE_UUID16_INIT(0x180A);
constexpr ble_uuid16_t DEVICE_MAKE_NAME_CHR_UUID16    = BLE_UUID16_INIT(0x2A29);
constexpr ble_uuid16_t DEVICE_MODEL_NUMBER_CHR_UUID16 = BLE_UUID16_INIT(0x2A24);
constexpr ble_uuid16_t DEVICE_FIRMWARE_VER_CHR_UUID16 = BLE_UUID16_INIT(0x2A26);

constexpr char ADC_FEED_SVC_UUID[]            = "e331016b-6618-4f8f-8997-1a2c7c9e5fa3";
constexpr char ADC_FEED_CHR_UUID[]            = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
constexpr char LC_CALIB_CHARACTERISTIC_UUID[] = "10adce11-68a6-450b-9810-ca11b39fd283";

constexpr char OTA_SVC_UUID[]         = "d6f1d96d-594c-4c53-b1c6-144a1dfde6d8";
constexpr char OTA_CONTROL_CHR_UUID[] = "7ad671aa-21c0-46a4-b722-270e3ae3d830";
constexpr char OTA_DATA_CHR_UUID[]    = "23408888-1f40-4cd8-9b89-ca8d45f8a5b0";

constexpr char toHex(uint8_t u) {
	if (u >= 0x0A)
		return u - 0x0A + 'a';
	return u + '0';
}

constexpr uint8_t fromHex(char x) {
	if (x >= '0' && x <= '9')
		return x - '0';
	if (x >= 'A' && x <= 'F')
		return x - 'A' + 10;
	if (x >= 'a' && x <= 'f')
		return x - 'a' + 10;
	return 0;
}

template <size_t LEN>
    requires(LEN == sizeof(ble_uuid128_t::value) * 2 + 4 + 1)
consteval ble_uuid128_t UUID128_FROM_STRING(const char (&uuid_s)[LEN]) {
	const size_t sz = sizeof(ble_uuid128_t::value);
	char         noDash[sz * 2]{};
	for (size_t i = 0, j = 0; i < LEN - 1; ++i) {
		if (uuid_s[i] != '-') {
			noDash[j] = uuid_s[i];
			j++;
		}
	}
	ble_uuid128_t uuid128{};
	uuid128.u = static_cast<ble_uuid_t>(BLE_UUID_TYPE_128);
	for (size_t i = 0; i < sz; ++i) {
		uuid128.value[sz - i - 1] = (fromHex(noDash[i * 2]) << 4) | fromHex(noDash[i * 2 + 1]);
	}
	return uuid128;
}

constexpr ble_uuid128_t ADC_FEED_SVC_UUID128 = UUID128_FROM_STRING(ADC_FEED_SVC_UUID);
constexpr ble_uuid128_t ADC_FEED_CHR_UUID128 = UUID128_FROM_STRING(ADC_FEED_CHR_UUID);
constexpr ble_uuid128_t LC_CALIB_CHARACTERISTIC_UUID128 =
    UUID128_FROM_STRING(LC_CALIB_CHARACTERISTIC_UUID);

constexpr ble_uuid128_t OTA_SVC_UUID128         = UUID128_FROM_STRING(OTA_SVC_UUID);
constexpr ble_uuid128_t OTA_CONTROL_CHR_UUID128 = UUID128_FROM_STRING(OTA_CONTROL_CHR_UUID);
constexpr ble_uuid128_t OTA_DATA_CHR_UUID128    = UUID128_FROM_STRING(OTA_DATA_CHR_UUID);

//======================== </UUIDs>

//======================== <OTA Update>

typedef uint8_t  OtaRequestType;
typedef uint8_t  OtaReplyType;
typedef uint32_t OtaFileSizeType;

constexpr OtaRequestType SVR_CHR_OTA_CONTROL_REQUEST = 1;
constexpr OtaRequestType SVR_CHR_OTA_CONTROL_DONE    = 4;

constexpr OtaReplyType SVR_CHR_OTA_CONTROL_NOP         = 0;
constexpr OtaReplyType SVR_CHR_OTA_CONTROL_REQUEST_ACK = 2;
constexpr OtaReplyType SVR_CHR_OTA_CONTROL_REQUEST_NAK = 3;
constexpr OtaReplyType SVR_CHR_OTA_CONTROL_DONE_ACK    = 5;
constexpr OtaReplyType SVR_CHR_OTA_CONTROL_DONE_NAK    = 6;

//======================== </OTA Update>

#endif // __dynamite_sampler_api_h__