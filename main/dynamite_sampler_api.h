#ifndef __dynamite_sampler_api_h__
#define __dynamite_sampler_api_h__

#include <stdint.h>

#ifndef __ORDER_BIG_ENDIAN__
#define __ORDER_BIG_ENDIAN__ 4321
#endif
#ifndef __ORDER_LITTLE_ENDIAN__
#define __ORDER_LITTLE_ENDIAN__ 1234
#endif

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

constexpr size_t DYNAMITE_NET_BYTE_ORDER = __ORDER_LITTLE_ENDIAN__;

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

#pragma pack(push, 1)

//======================== <ADC Data>

struct AdcFeedNetworkData {
	static constexpr size_t DATA_BYTE_ORDER = __ORDER_BIG_ENDIAN__;
	struct AdcSample {
		static constexpr size_t BYTES_PER_SAMPLE = 3;

		uint8_t sample[BYTES_PER_SAMPLE];
	};

	static constexpr size_t NUM_CHAN = 4;

	AdcSample chan[NUM_CHAN];
};

struct AdcFeedNetworkPacket {
	static constexpr size_t BLE_PAYLOAD_SZ = 244;

	struct Header {
		uint16_t sample_sequence_number;
	};
	Header                  hrd;
	static constexpr size_t NUM_SAMPLES =
	    (BLE_PAYLOAD_SZ - sizeof(Header)) / sizeof(AdcFeedNetworkData);
	AdcFeedNetworkData adc[NUM_SAMPLES];
};

struct AdcConfigNetworkData {
	uint8_t  version;
	uint16_t id;
	uint16_t status;
	uint16_t mode;
	uint16_t clock;
	uint16_t pga;
};

//======================== </ADC Data>

// A large arbirtary size of the partition to read. It will hold all the possible data.
// Its made to be large enough so that the format can change without changing the firmware.
// It can't be larger than the max BLE characteristic length of 512 (0x200)
struct CalibrationNetworkData {
	static constexpr size_t CALIB_PARTITION_LENGTH = 0xff;
	static_assert(CALIB_PARTITION_LENGTH <= 512);

	uint8_t data[CALIB_PARTITION_LENGTH];
};

// BLE Standard
struct TxPowerNetworkData {
	int8_t val;
};

#pragma pack(pop)

#endif // __dynamite_sampler_api_h__
