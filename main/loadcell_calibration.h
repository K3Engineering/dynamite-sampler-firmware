#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool initCalibrationStorage();
bool readCalibrationN(const char *cmd, char *reply, size_t replyLen);
bool writeCalibrationKeyVal(const char *cmd);
bool readCalibrationKey(const char *cmd, char *reply, size_t replyLen);
bool deleteCalibrationKey(const char *cmd);

#endif // _LOADCELL_CALIBRATION_H
