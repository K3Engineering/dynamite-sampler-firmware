#ifndef _BLE_OTA_INTERFACE_H
#define _BLE_OTA_INTERFACE_H

#include <NimBLEServer.h>

void *otaControlPtr();
void otaOnDisconnect();
void otaOnControlSubscribe();

#endif // _BLE_OTA_INTERFACE_H
