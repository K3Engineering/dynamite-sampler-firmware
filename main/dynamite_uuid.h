#ifndef __dynamite_uuid_h__
#define __dynamite_uuid_h__

#include <host/ble_uuid.h>
#include <stdint.h>

#include "dynamite_sampler_api.h"

constexpr uint8_t fromHex(char x) {
	if (x >= '0' && x <= '9')
		return x - '0';
	if (x >= 'A' && x <= 'F')
		return x - 'A' + 10;
	if (x >= 'a' && x <= 'f')
		return x - 'a' + 10;
	return 0;
}

consteval ble_uuid128_t UUID128_FROM_STRING(const char (&uuid_s)[BLE_UUID_STR_LEN]) {
	const size_t sz = sizeof(ble_uuid128_t::value);
	char         noDash[sz * 2]{};
	for (size_t i = 0, j = 0; i < sizeof(uuid_s) - 1; ++i) {
		if (uuid_s[i] != '-') {
			noDash[j] = uuid_s[i];
			j++;
		}
	}
	ble_uuid128_t uuid128{};
	uuid128.u.type = BLE_UUID_TYPE_128;
	for (size_t i = 0; i < sz; ++i) {
		uuid128.value[sz - i - 1] = (fromHex(noDash[i * 2]) << 4) | fromHex(noDash[i * 2 + 1]);
	}
	return uuid128;
}

constexpr ble_uuid16_t DEVICE_INFO_SVC_UUID16      = BLE_UUID16_INIT(DEVICE_INFO_SVC_UUID);
constexpr ble_uuid16_t DEVICE_MAKE_NAME_CHR_UUID16 = BLE_UUID16_INIT(DEVICE_MAKE_NAME_CHR_UUID);
constexpr ble_uuid16_t DEVICE_FIRMWARE_VER_CHR_UUID16 =
    BLE_UUID16_INIT(DEVICE_FIRMWARE_VER_CHR_UUID);
constexpr ble_uuid16_t DEVICE_TX_POWER_CHR_UUID16 = BLE_UUID16_INIT(DEVICE_TX_POWER_CHR_UUID);

constexpr ble_uuid128_t DYNAMITE_SAMPLER_SVC_UUID128 =
    UUID128_FROM_STRING(DYNAMITE_SAMPLER_SVC_UUID);
constexpr ble_uuid128_t ADC_FEED_CHR_UUID128 = UUID128_FROM_STRING(ADC_FEED_CHR_UUID);
constexpr ble_uuid128_t LC_CALIB_CHR_UUID128 = UUID128_FROM_STRING(LC_CALIB_CHR_UUID);
constexpr ble_uuid128_t ADC_CONF_CHR_UUID128 = UUID128_FROM_STRING(ADC_CONF_CHR_UUID);

constexpr ble_uuid128_t OTA_SVC_UUID128         = UUID128_FROM_STRING(OTA_SVC_UUID);
constexpr ble_uuid128_t OTA_CONTROL_CHR_UUID128 = UUID128_FROM_STRING(OTA_CONTROL_CHR_UUID);
constexpr ble_uuid128_t OTA_DATA_CHR_UUID128    = UUID128_FROM_STRING(OTA_DATA_CHR_UUID);

constexpr ble_uuid128_t TX_PWR_SVC_UUID128 = UUID128_FROM_STRING(TX_PWR_SVC_UUID);
constexpr ble_uuid128_t TX_PWR_CHR_UUID128 = UUID128_FROM_STRING(TX_PWR_CHR_UUID);

#endif
