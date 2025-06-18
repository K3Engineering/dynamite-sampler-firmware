#ifndef _BLE_PROC_H
#define _BLE_PROC_H

void setupBle(int core);
void otaConditionalRollback(void);
void setAdcConfigCharacteristic(const uint8_t *data, const size_t length);

#endif // _BLE_PROC_H
