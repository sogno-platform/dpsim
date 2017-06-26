#include <iostream>
#include <string>
#include "NetlistSim.h"
#include "Examples/SynchronGenUnitTest.h"
#include "Examples/SimpleCircuitTest.h"
#include "Examples/ReferenceCircuits.h"
#include "Examples/ShmemTest.h"
#include <complex>

using namespace DPsim;

int main(int argc, char* argv[]) {



	shmemRTExample();
	//shmemDistributedExample(argc, argv);
	//shmemDistributedRef();
	/*simulationExample1();
	simulationExample2();
	simulationExample3();
	simulationExampleIdealVS();
	simulationExampleIdealVS2();
	simulationExampleIdealVS3();
	simulationExampleRXLine3();
	simulationExampleRXLine();
	simulationExampleRXLine2();
	simulationExamplePiLine();
	simulationExamplePiLine2();*/

	//simulationExample1();
	//simulationExample2();
	//simulationExample3();
	//simulationExampleIdealVS();
	//simulationExampleIdealVS2();
	//simulationExampleIdealVS3();
	//simulationExampleRXLine3();
	//simulationExampleRXLine();
	//simulationExampleRXLine2();

	//NetlistSim(argc, argv);
	
	//SynGenUnitTestBalancedResLoad();
	//SynGenUnitTestPhaseToPhaseFault();
	//SynGenUnitTestThreePhaseFault();	
	//SynGenDPUnitTestBalancedResLoad();

	//RXLineResLoad();
	//VarFreqRXLineResLoad();
	//RXLineResLoadEMT();
	//VarFreqRXLineResLoadEMT();
	
	//runDpEmtVarFreqStudy();

	//std::cin.get();
	return 0;	
}

