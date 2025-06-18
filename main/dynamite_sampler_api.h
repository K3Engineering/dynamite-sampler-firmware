#ifndef __dynamite_sampler_api_h__
#define __dynamite_sampler_api_h__

#include <stdint.h>

constexpr char DEVICE_MANUFACTURER_NAME[] = "K3 Engineering";

//======================== <UUIDs>

// Bluetooth assigned UUIDs
// https://www.bluetooth.com/wp-content/uploads/Files/Specification/Assigned_Numbers.html
constexpr uint16_t DEVICE_INFO_SVC_UUID         = 0x180A;
constexpr uint16_t DEVICE_MAKE_NAME_CHR_UUID    = 0x2A29;
constexpr uint16_t DEVICE_FIRMWARE_VER_CHR_UUID = 0x2A26;
constexpr uint16_t DEVICE_TX_POWER_CHR_UUID     = 0x2A07;

// Dynamite sampler specific UUIDs
// TODO: the ADC_FEED_CHR_UUID & OTA UUIDs were copied from sample code originally.
// They should be changed to be unique.
constexpr char DYNAMITE_SAMPLER_SVC_UUID[] = "e331016b-6618-4f8f-8997-1a2c7c9e5fa3";
constexpr char ADC_FEED_CHR_UUID[]         = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
constexpr char LC_CALIB_CHR_UUID[]         = "10adce11-68a6-450b-9810-ca11b39fd283";
constexpr char ADC_CONF_CHR_UUID[]         = "adcc0f19-2575-4502-9a48-0e99974eb34f";

constexpr char OTA_SVC_UUID[]         = "d6f1d96d-594c-4c53-b1c6-144a1dfde6d8";
constexpr char OTA_CONTROL_CHR_UUID[] = "7ad671aa-21c0-46a4-b722-270e3ae3d830";
constexpr char OTA_DATA_CHR_UUID[]    = "23408888-1f40-4cd8-9b89-ca8d45f8a5b0";

// The TX characteristic if for setting only. Use the device info characteristic to
// read the current power.
constexpr char TX_PWR_SVC_UUID[] = "74788a4c-72aa-4180-a478-59e969b959c9";
constexpr char TX_PWR_CHR_UUID[] = "7478c418-35d3-4c3d-99d9-2de090159664";

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

//======================== <ADC Data>
// These structs are not used. Currently for documenting the format.
// TODO: expose the format in a nicer way.
#pragma pack(push, 1)

struct DynamiteAdcSample {
	static constexpr size_t SAMPLE_BITS  = 24;
	static constexpr size_t SAMPLE_BYTES = (SAMPLE_BITS + 7) / 8;

	uint8_t sample[SAMPLE_BYTES];
};

struct DynamiteFeedData {
	static constexpr size_t NUM_CHANELS = 4;

	uint16_t          status;
	DynamiteAdcSample data[NUM_CHANELS];
	uint8_t           crc;
};

struct DynamiteFeedChunk {
	DynamiteFeedData data[16];
};

#pragma pack(pop)
//======================== </ADC Data>

#endif // __dynamite_sampler_api_h__
