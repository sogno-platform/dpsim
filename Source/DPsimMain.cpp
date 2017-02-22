#include <iostream>
#include <string>
#include "NetlistSim.h"
#include "SynchronGenUnitTest.h"
#include <complex>

int main(int argc, char* argv[]) {

	NetlistSim(argc, argv);
	
	//SynGenUnitTestBalancedResLoad();
	//SynGenUnitTestPhaseToPhaseFault();
	//SynGenUnitTestThreePhaseFault();
	
	SynGenDPUnitTestBalancedResLoad();

	std::cin.get();
	return 0;	
}

