#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool initCalibrationStorage();
bool readCalibrationN(CalibrationNetworkData *data);
bool writeCalibrationKeyVal(CalibrationNetworkData *data);
bool readCalibrationKeyVal(CalibrationNetworkData *data);
bool deleteCalibrationKey(CalibrationNetworkData *data);

#endif // _LOADCELL_CALIBRATION_H
