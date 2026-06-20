#ifndef _ADC_PROC_H
#define _ADC_PROC_H

#include "dynamite_sampler_api.h"

void setupAdc(int core);
bool startAdcAcquisition();
void stopAdcAcquisition();
void setAdcExternalReference(bool enable);

const AdcConfigNetworkData getAdcConfig();

#endif // _ADC_PROC_H
