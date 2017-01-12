#include <iostream>
#include <string>
#include "NetlistSim.h"
#include "SysGenUnitTest.h"

int main(int argc, char* argv[]) {

	//NetlistSim(argc, argv);
	
	//SysGenUnitTestBalancedResLoad();
	SysGenUnitTestResLoadAndFault();

	std::cin.get();
	return 0;	
}

