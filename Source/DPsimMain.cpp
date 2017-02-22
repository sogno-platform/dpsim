#include <iostream>
#include <string>
#include "NetlistSim.h"
#include "SynchronGenUnitTest.h"

int main(int argc, char* argv[]) {

	NetlistSim(argc, argv);
	
	//SynGenUnitTestBalancedResLoad();
	//SynGenUnitTestPhaseToPhaseFault();
	//SynGenUnitTestThreePhaseFault();

	std::cin.get();
	return 0;	
}

