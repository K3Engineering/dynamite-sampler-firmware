#ifndef _BLE_OTA_INTERFACE_H
#define _BLE_OTA_INTERFACE_H

#include <NimBLEServer.h>

void setupBleOta(NimBLEServer *server);
void setDeviceInfo(NimBLEServer *server);

#endif // _BLE_OTA_INTERFACE_H
