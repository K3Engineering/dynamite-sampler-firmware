#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool initCalibrationStorage();
bool readCalibrationN(const char *cmd, size_t cmdLen, char *reply, size_t replyLen);
bool writeCalibrationKeyVal(const char *cmd, size_t cmdLen);
bool readCalibrationKey(const char *cmd, size_t cmdLen, char *reply, size_t replyLen);
bool deleteCalibrationKey(const char *cmd, size_t cmdLen);

#endif // _LOADCELL_CALIBRATION_H
