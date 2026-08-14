#ifndef _ADC_BLE_INTERFACE_H
#define _ADC_BLE_INTERFACE_H

#include <freertos/stream_buffer.h>

#include "dynamite_sampler_api.h"

extern StreamBufferHandle_t adcStreamBufferHandle;
extern bool startupDiagnosticIsOk;

// DLE: Data Length Extension allows the Data Channel payload size to be increased from 27 to 251
// bytes. We want the largest possible size to send as much data in a single transmission.
constexpr uint16_t DLE_MAX_SIZE = 251;

constexpr uint16_t L2CAP_HEADER_SIZE = 4;
constexpr uint16_t ATT_HEADER_SIZE = 3;

// The size of the max ATT payload is the data channel payload minus the headers.
// We set the maximum payload size using DLE (251 bytes).
// L2CAP header (4 bytes) and ATT header (3 bytes).
// ATT size is 251-4. ATT payload is 251-4-3.
constexpr uint16_t ATT_PAYLOAD_MAX_SIZE = DLE_MAX_SIZE - L2CAP_HEADER_SIZE - ATT_HEADER_SIZE;

// Note: On iOS and macOS, the link layer can be set to 251, but the maximum ATT size will be 185
// bytes. The ATT payload will be the ATT size minus the ATT header 185-3=182
// TODO: the actual payload size should be calculated

constexpr size_t ADC_FEED_MAX_CHUNK_SZ =
    (ATT_PAYLOAD_MAX_SIZE / sizeof(AdcFeedNetworkData)) * sizeof(AdcFeedNetworkData);
static_assert(ADC_FEED_MAX_CHUNK_SZ <= ATT_PAYLOAD_MAX_SIZE);

#endif // _ADC_BLE_INTERFACE_H
