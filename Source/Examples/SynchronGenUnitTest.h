#ifndef  SYNCHRONGENUNITTEST_H
#define SYNCHRONGENUNITTEST_H

#include "../MathLibrary.h"

namespace DPsim {
	// EMT generator tests
	void SynGenUnitTestBalancedResLoad();
	void SynGenUnitTestPhaseToPhaseFault();
	void SynGenUnitTestThreePhaseFault();

	// Dynamic Phasor generator tests
	void SynGenDPUnitTestBalancedResLoad();
}

#endif
