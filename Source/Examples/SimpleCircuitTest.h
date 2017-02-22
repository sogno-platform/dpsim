#ifndef SIMPLECIRCUITTEST_H
#define SIMPLECIRCUITTEST_H

#include "../MathLibrary.h"

void RXLineResLoad();
void VarFreqRXLineResLoad(DPsim::Real timeStep, DPsim::Real finalTime, DPsim::Real freqStep, DPsim::Real loadStep, DPsim::Real rampTime);
void RXLineResLoadEMT();
void VarFreqRXLineResLoadEMT(DPsim::Real timeStep, DPsim::Real finalTime, DPsim::Real freqStep, DPsim::Real loadStep, DPsim::Real rampTime);
void runDpEmtVarFreqStudy();

#endif
