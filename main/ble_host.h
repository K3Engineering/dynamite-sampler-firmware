#ifndef _BLE_HOST_H
#define _BLE_HOST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// A BLE server built directly on the NimBLE C host. The GATT layout is a
// static table; each characteristic is a set of plain function hooks moving
// byte spans with explicit lengths.
//
// Every service and characteristic except OTA is described. Nothing calls
// this yet, so it is compiled and linked but unreferenced.

// Raised into the application from the NimBLE host task. None may block.
struct BleServerEvents {
	void (*onConnect)(uint16_t connHandle);
	void (*onDisconnect)(int reason);
	// The CCCD's notify bit, after the client's write, for valHandle.
	void (*onSubscribe)(uint16_t valHandle, bool notifyEnabled);
	// The peer's TX octet count once Data Length Extension is negotiated.
	void (*onDataLengthChange)(uint16_t maxTxOctets);
};

// Bring up the controller and host, register the GATT server, and start
// advertising. Returns false if the host or the GATT registration failed.
bool bleHostStart(const char *deviceName, const BleServerEvents &events);

// Value handle of the ADC feed characteristic, so the streaming task can
// push notifications through bleNotify.
uint16_t bleAdcFeedHandle(void);

// Notify subscribers of one characteristic. Returns false when the stack
// rejects it (no connection, no subscribers, no free mbuf), so the caller
// counts drops instead of losing samples silently.
bool bleNotify(uint16_t valHandle, const void *data, size_t len);

#endif // _BLE_HOST_H
