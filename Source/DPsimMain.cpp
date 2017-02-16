#include <iostream>
#include <string>
#include "NetlistSim.h"
#include "SynchronGenUnitTest.h"
#include "SimpleCircuitUnitTest.h"
#include <complex>

int main(int argc, char* argv[]) {

	//NetlistSim(argc, argv);
	
	//SynGenUnitTestBalancedResLoad();
	//SynGenUnitTestPhaseToPhaseFault();
	//SynGenUnitTestThreePhaseFault();	
	//SynGenDPUnitTestBalancedResLoad();

	//RXLineResLoad();
	VarFreqRXLineResLoad();

	//std::cin.get();
	return 0;	
}

