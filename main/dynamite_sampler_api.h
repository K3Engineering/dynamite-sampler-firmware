#ifndef __dynamite_sampler_api_h__
#define __dynamite_sampler_api_h__

#include <stdint.h>

//======================== <UUIDs>

constexpr uint16_t DEVICE_INFO_SVC_UUID         = 0x180A;
constexpr uint16_t DEVICE_MAKE_NAME_CHR_UUID    = 0x2A29;
constexpr uint16_t DEVICE_MODEL_NUM_CHR_UUID    = 0x2A24;
constexpr uint16_t DEVICE_FIRMWARE_VER_CHR_UUID = 0x2A26;

constexpr char ADC_FEED_SVC_UUID[]            = "e331016b-6618-4f8f-8997-1a2c7c9e5fa3";
constexpr char ADC_FEED_CHR_UUID[]            = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
constexpr char LC_CALIB_CHARACTERISTIC_UUID[] = "10adce11-68a6-450b-9810-ca11b39fd283";

constexpr char OTA_SVC_UUID[]         = "d6f1d96d-594c-4c53-b1c6-144a1dfde6d8";
constexpr char OTA_CONTROL_CHR_UUID[] = "7ad671aa-21c0-46a4-b722-270e3ae3d830";
constexpr char OTA_DATA_CHR_UUID[]    = "23408888-1f40-4cd8-9b89-ca8d45f8a5b0";

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
#pragma pack(push, 1)

struct DynamiteAdcSample {
	static constexpr size_t BITS_PER_SAMPLE = 24;

	uint8_t val[BITS_PER_SAMPLE / 8];
};

struct DynamiteAdcFeedData {
	static constexpr size_t NUM_CHANELS = 4;

	uint16_t          status;
	DynamiteAdcSample data[NUM_CHANELS];
	uint8_t           crc;
};

struct DynamiteAdcFeedChunk {
	static constexpr size_t SZ = 16;

	DynamiteAdcFeedData data[SZ];
};

#pragma pack(pop)
//======================== </ADC Data>

#endif // __dynamite_sampler_api_h__