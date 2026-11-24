#ifndef BRIDGE_H
#define BRIDGE_H
#include "sim.h"

int BridgeAbs(const float *in, float *out, void *user);   // AbsFn calling Module.absStep
void BridgeAbsReset(void);                                // clears the controller's mem

#endif
