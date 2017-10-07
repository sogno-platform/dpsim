#ifndef SIMPLECIRCUIT_H
#define SIMPLECIRCUIT_H

#include "../MathLibrary.h"

namespace DPsim {
	void RXLineResLoad();
	void VarFreqRXLineResLoad(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void RXLineResLoadEMT();
	void VarFreqRXLineResLoadEMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void runDpEmtVarFreqStudy();	
	void RTExample();

	void VarFreqRXLineResLoad_NZ_Paper(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void VarFreqRXLineResLoadEMT_NZ_Paper(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void runDpEmtVarFreqStudy_NZ_Paper();
	
};

#endif
