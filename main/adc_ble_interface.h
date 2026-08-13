#ifndef _ADC_BLE_INTERFACE_H
#define _ADC_BLE_INTERFACE_H

#include <freertos/stream_buffer.h>

#include "dynamite_sampler_api.h"

extern StreamBufferHandle_t adcStreamBufferHandle;
extern bool startupDiagnosticIsOk;

// Nimble creates a GATT connection, which has some overhead.
constexpr uint16_t BLE_PUBL_DATA_DLE         = 251;
constexpr uint16_t BLE_PUBL_DATA_ATT_PAYLOAD = BLE_PUBL_DATA_DLE - 4 - 3;

constexpr size_t ADC_FEED_CHUNK_SZ =
    (BLE_PUBL_DATA_ATT_PAYLOAD / sizeof(AdcFeedNetworkData)) * sizeof(AdcFeedNetworkData);
static_assert(ADC_FEED_CHUNK_SZ <= BLE_PUBL_DATA_ATT_PAYLOAD);

// Smallest N that still carries 1 ksps in one notify per 15 ms connection event.
// ATT MTU 185 => payload 182 => 15 samples + 2-byte header.
constexpr size_t ADC_FEED_MIN_SAMPLES = 15;
static_assert(ADC_FEED_MIN_SAMPLES * sizeof(AdcFeedNetworkData) +
                  sizeof(AdcFeedNetworkPacket::Header) + 3 ==
              185);

// Samples per notify for the current stream. 0 when not streaming. Frozen at subscribe.
extern volatile size_t adcFeedNumSamples;

#endif // _ADC_BLE_INTERFACE_H
