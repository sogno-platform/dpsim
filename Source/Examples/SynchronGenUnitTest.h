#ifndef SYNCHRONGENUNITTEST_H
#define SYNCHRONGENUNITTEST_H

#include "../MathLibrary.h"

namespace DPsim {
	// EMT generator tests
	void SynGenUnitTestBalancedResLoad();
	void SynGenUnitTestPhaseToPhaseFault();
	void SynGenUnitTestThreePhaseFault();

	// Test generator with load also in dq. 
	void SynGenUnitTestdqBalancedResLoad();

	// Dynamic Phasor generator tests
	void SynGenDPUnitTestBalancedResLoad();
	void SynGenDPUnitTestThreePhaseFault();
}

#endif
