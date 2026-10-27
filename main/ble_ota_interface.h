#ifndef _BLE_OTA_INTERFACE_H
#define _BLE_OTA_INTERFACE_H

#include <NimBLEServer.h>

void *otaControlPtr();
void otaOnDisconnect(void);
void otaOnControlSubscribe(void *control_);

#endif // _BLE_OTA_INTERFACE_H
