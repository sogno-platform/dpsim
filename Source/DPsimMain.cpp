#include <iostream>
#include <string>
#include "NetlistSim.h"
#include "Examples/Testing.h"
#include <complex>

using namespace DPsim;

int main(int argc, char* argv[]) {	

	// Netlist interpreter - deprecated
	//NetlistSim(argc, argv);

	// Testing area
	runTest();	

	//std::cin.get();
	return 0;	
}

