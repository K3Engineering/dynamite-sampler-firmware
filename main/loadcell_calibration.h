#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool readLoadcellCalibration(CalibrationNetworkData *calibration);

bool writeLoadcellStr2(const char *keyVal);
bool readLoadcellCalibration2(CalibrationNetworkData *calibration);

#endif // _LOADCELL_CALIBRATION_H
