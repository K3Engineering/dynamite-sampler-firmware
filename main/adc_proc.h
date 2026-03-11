#ifndef _ADC_PROC_H
#define _ADC_PROC_H

#include "dynamite_sampler_api.h"

void setupAdc(int core);
void startAdcAcquisition();
void stopAdcAcquisition();

const AdcConfigNetworkData getAdcConfig();

#endif // _ADC_PROC_H
