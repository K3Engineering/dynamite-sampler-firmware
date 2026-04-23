#ifndef _BLE_PROC_H
#define _BLE_PROC_H

#include <stdint.h>

enum struct DeviceLock : uint8_t {
	Open      = 0,
	Streaming = 1,
	Ota       = 2,
};

extern DeviceLock deviceLock;

void setupBle(int core);
void otaConditionalRollback(void);

#endif // _BLE_PROC_H
