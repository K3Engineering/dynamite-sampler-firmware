#ifndef _BLE_PROC_H
#define _BLE_PROC_H

void setupBle(int core, const uint8_t *calibrationData, const size_t calibrationLength);
void otaConditionalRollback(void);

#endif // _BLE_PROC_H
