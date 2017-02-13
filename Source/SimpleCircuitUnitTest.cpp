#include "SimpleCircuitUnitTest.h"

#include "Components.h"
#include "Logger.h"
#include "Simulation.h"

void RXLineResLoad() {
	// Define Object for saving data on a file
	Logger log, vtLog, jLog;

	// Declare circuit components
	
	double loadRes = 1037.8378;

	std::vector<BaseComponent*> circElements;
	circElements.push_back(new LinearResistor("r1", 0, 1, loadRes));

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.01; dt = 0.000001; t = 0;
	Simulation newSim(circElements, om, dt, tf, log);
			
	// Main Simulation Loop
	while (newSim.mTime < tf)
	{
		// Set to zero because all components will add their contribution for the current time step to the current value
		newSim.mRightSideVector.setZero();

		// Execute step for all circuit components
		for (std::vector<BaseComponent*>::iterator it = newSim.mElements.begin(); it != newSim.mElements.end(); ++it) {
			(*it)->step(newSim.mSystemMatrix, newSim.mRightSideVector, newSim.mCompOffset, om, dt, newSim.mTime);
		}
				
		// Execute PostStep for all components, generator states are recalculated based on new terminal voltage
		for (std::vector<BaseComponent*>::iterator it = newSim.mElements.begin(); it != newSim.mElements.end(); ++it) {
			(*it)->postStep(newSim.mSystemMatrix, newSim.mRightSideVector, newSim.mLeftSideVector, newSim.mCompOffset, om, dt, newSim.mTime);
		}

		// Save simulation step data		
		vtLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getLeftSideVector()).str();
		jLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getRightSideVector()).str();

		std::cout << newSim.getTime() << std::endl;

		// timestep
		newSim.mTime += dt;
	}

	// Write simulation data to file
	log.WriteLogToFile("log.txt");
	vtLog.WriteLogToFile("data_vt.csv");
	jLog.WriteLogToFile("data_j.csv");

	std::cout << "Simulation finished." << std::endl;
}