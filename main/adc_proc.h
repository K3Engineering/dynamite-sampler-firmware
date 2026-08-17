#ifndef _ADC_PROC_H
#define _ADC_PROC_H

#include "dynamite_sampler_api.h"

void setupAdc(int core);
bool startAdcAcquisition(size_t interval);
void stopAdcAcquisition();

const AdcConfigNetworkData getAdcConfig();

#endif // _ADC_PROC_H
