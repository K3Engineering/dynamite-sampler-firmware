#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool initCalibrationStorage();
bool readCalibrationAll(CalibrationNetworkData *calibration);
bool writeCalibrationKeyVal(const void *data, size_t len);
bool deleteCalibrationKey(const void *data, size_t len);

#endif // _LOADCELL_CALIBRATION_H
