#ifndef TESTING_H
#define TESTING_H

#include "SynchronGenUnitTest.h"
#include "SyncGenUnitTestVBR.h"
#include "ReferenceCircuits.h"
#include "ShmemTest.h"
#include "SimpleCircuitTest.h"

namespace DPsim {

	void runTest() {
		
		// #### Shared memory interface tests ################
		//shmemRTExample();
		//shmemDistributed(argc, argv);
		//shmemDistributedRef();

		// #### Reference circuits test ################
		//simulationExample1();
		//simulationExample2();
		//simulationExample3();
		//simulationExampleIdealVS();
		//simulationExampleIdealVS2();
		//simulationExampleIdealVS3();
		//simulationExampleRXLine3();
		//simulationExampleRXLine();
		//simulationExampleRXLine2();
		//simulationExamplePiLine();
		//simulationExamplePiLine2();
		//simulationExample1();
		//simulationExample2();
		//simulationExample3();
		//simulationExampleIdealVS();
		//simulationExampleIdealVS2();
		//simulationExampleIdealVS3();
		//simulationExampleRXLine3();
		//simulationExampleRXLine();
		//simulationExampleRXLine2();

		// #### Synchronous generator unit tests ################
		// EMT classic dq model
		SynGenUnitTestBalancedResLoad();
		//SynGenUnitTestPhaseToPhaseFault();
		//SynGenUnitTestThreePhaseFault();	
		
		// DP classic dq model
		//SynGenDPUnitTestBalancedResLoad();

		// EMT VBR model
		//SynGenUnitTestVBR();

		// #### Variable frequency tests ################
		//RXLineResLoad();
		//VarFreqRXLineResLoad();
		//RXLineResLoadEMT();
		//VarFreqRXLineResLoadEMT();				
		//runDpEmtVarFreqStudy();
		//runDpEmtVarFreqStudy_NZ_Paper();
	}
}

#endif