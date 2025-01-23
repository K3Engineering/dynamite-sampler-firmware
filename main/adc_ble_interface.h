#ifndef _ADC_BLE_INTERFACE_H
#define _ADC_BLE_INTERFACE_H

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include <string.h>

// Nimble creates a GATT connection, which has some overhead.
constexpr uint16_t BLE_PUBL_DATA_DLE         = 251;
constexpr uint16_t BLE_PUBL_DATA_ATT_PAYLOAD = BLE_PUBL_DATA_DLE - 4 - 3;

#pragma pack(push, 1)
struct BleAdcFeedData {
	static constexpr size_t DATA_SIZE = 3 * 4;

	uint16_t status;
	uint8_t  data[DATA_SIZE];
	uint8_t  crc;

	BleAdcFeedData(uint16_t status_, const void *data_, uint8_t crc_) : status{status_}, crc{crc_} {
		memcpy(data, data_, sizeof(data));
	}
};
#pragma pack(pop)

constexpr size_t ADC_FEED_CHUNK_SZ =
    (BLE_PUBL_DATA_ATT_PAYLOAD / sizeof(BleAdcFeedData)) * sizeof(BleAdcFeedData);
static_assert(ADC_FEED_CHUNK_SZ <= BLE_PUBL_DATA_ATT_PAYLOAD);

struct BleAccess {
	StreamBufferHandle_t adcStreamBufferHandle;
	TaskHandle_t         bleAdcFeedPublisherTaskHandle;

	bool deviceConnected;
};

extern BleAccess bleAccess;

#endif // _ADC_BLE_INTERFACE_H
