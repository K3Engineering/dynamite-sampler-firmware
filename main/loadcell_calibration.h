#ifndef _LOADCELL_CALIBRATION_H
#define _LOADCELL_CALIBRATION_H

#include "dynamite_sampler_api.h"

bool readLoadcellCalibration(CalibrationNetworkData *calibration);

bool writeLoadcellVal(const uint8_t *data, size_t len);
bool readLoadcellCalibration2(CalibrationNetworkData *calibration);

#endif // _LOADCELL_CALIBRATION_H
