#ifndef _BLE_PROC_H
#define _BLE_PROC_H

void setupBle(int core, uint8_t *calibrationData, size_t calibrationLength);
void otaConditionalRollback(void);

#endif // _BLE_PROC_H
