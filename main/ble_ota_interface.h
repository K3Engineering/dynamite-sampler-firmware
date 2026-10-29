#ifndef _BLE_OTA_INTERFACE_H
#define _BLE_OTA_INTERFACE_H

#include "host/ble_gatt.h"

const ble_gatt_svc_def *otaServiceDef();
void otaOnDisconnect();
void otaOnSubscribe(uint16_t attrHandle);

#endif // _BLE_OTA_INTERFACE_H
