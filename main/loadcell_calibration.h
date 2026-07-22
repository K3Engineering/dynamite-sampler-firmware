#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool initUserKeyValStorage();
bool readDeviceByIdx(const char *cmd, char *reply, size_t replySz);
bool writeDeviceKeyVal(const char *cmd);
bool readDeviceKey(const char *cmd, char *reply, size_t replySz);
bool deleteDeviceKey(const char *cmd);

#endif // _LOADCELL_CALIBRATION_H
