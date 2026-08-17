#ifndef _ADC_BLE_INTERFACE_H
#define _ADC_BLE_INTERFACE_H

#include <freertos/stream_buffer.h>

#include "dynamite_sampler_api.h"

extern StreamBufferHandle_t adcStreamBufferHandle;
extern bool startupDiagnosticIsOk;

// BLE can transmit a certain size over the air in a single transmission (Link Layer or LL). This is
// controlled by DLE. There is also the ATT payload size. If the ATT payload size is larger than the
// LL payload, the information will be transmitted as several packets. BLE packets have a lot of
// overhead. We want to maximize through put, and the easiest way is to make our packets smaller
// than the LL payload so that they are never split apart. Below is the calculation for the maximum
// amount of data we can send in a single LL packet.

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

constexpr size_t ADC_FEED_MAX_CHUNK_SZ =
    (ATT_PAYLOAD_MAX_SIZE / sizeof(AdcFeedNetworkData)) * sizeof(AdcFeedNetworkData);
static_assert(ADC_FEED_MAX_CHUNK_SZ <= ATT_PAYLOAD_MAX_SIZE);

#endif // _ADC_BLE_INTERFACE_H
