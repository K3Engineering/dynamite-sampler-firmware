#ifndef _ADC_PROC_H
#define _ADC_PROC_H

#include "dynamite_sampler_api.h"

void setupAdc(int core);
bool startAdcAcquisition();
void stopAdcAcquisition();

const AdcConfigNetworkData getAdcConfig();
const AdcMuxNetworkData getAdcMux();
bool setAdcMux(uint8_t channel, uint8_t mux);

#endif // _ADC_PROC_H
