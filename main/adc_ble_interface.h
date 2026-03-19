#ifndef _ADC_BLE_INTERFACE_H
#define _ADC_BLE_INTERFACE_H

#include <freertos/stream_buffer.h>

#include "dynamite_sampler_api.h"

extern StreamBufferHandle_t adcStreamBufferHandle;

// Nimble creates a GATT connection, which has some overhead.
constexpr uint16_t BLE_PUBL_DATA_DLE         = 251;
constexpr uint16_t BLE_PUBL_DATA_ATT_PAYLOAD = BLE_PUBL_DATA_DLE - 4 - 3;

constexpr size_t ADC_FEED_CHUNK_SZ =
    (BLE_PUBL_DATA_ATT_PAYLOAD / sizeof(AdcFeedNetworkData)) * sizeof(AdcFeedNetworkData);
static_assert(ADC_FEED_CHUNK_SZ <= BLE_PUBL_DATA_ATT_PAYLOAD);

#endif // _ADC_BLE_INTERFACE_H
