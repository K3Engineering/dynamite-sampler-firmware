#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool readCalibrationAll(CalibrationNetworkData *calibration);
bool writeCalibrationKeyVal(const uint8_t *data, size_t len);
bool deleteCalibrationKey(const uint8_t *data, size_t len);

#endif // _LOADCELL_CALIBRATION_H
