#include <iostream>
#include <string>
#include "NetlistSim.h"
#include "Examples/SynchronGenUnitTest.h"
#include "Examples/SyncGenUnitTestVBR.h"
#include "Examples/ReferenceCircuits.h"
#include "Examples/ShmemTest.h"
#include <complex>

using namespace DPsim;

int main(int argc, char* argv[]) {



	//shmemRTExample();
	//shmemDistributed(argc, argv);
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
	//SynGenUnitTestdqBalancedResLoad();

	//SynGenUnitTestPhaseToPhaseFault();
	//SynGenUnitTestThreePhaseFault();	
	//SynGenDPUnitTestBalancedResLoad();
	SynGenDPUnitTestThreePhaseFault();
	//SynGenUnitTestVBR();

	//RXLineResLoad();
	//VarFreqRXLineResLoad();
	//RXLineResLoadEMT();
	//VarFreqRXLineResLoadEMT();
	
	//runDpEmtVarFreqStudy();

	//std::cin.get();
	return 0;	
}

