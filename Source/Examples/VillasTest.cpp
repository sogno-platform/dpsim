#include "VillasTest.h"
#include "../Simulation.h"
#include "../VillasInterface.h"
#include "../Utilities.h"

using namespace DPsim;

void DPsim::villasExample()
{
	// Very simple test circuit. Just 2 resistors and a current read from VILLASnode.
	Logger log, llog, rlog;
	std::vector<BaseComponent*> comps;

	ExternalVoltageSource *evs = new ExternalVoltageSource("v_s", 1, 0, 1);
	comps.push_back(evs);
	comps.push_back(new LinearResistor("r_s", 1, 2, 1));
	comps.push_back(new LinearResistor("r_line", 2, 3, 1));
	comps.push_back(new Inductor("l_line", 3, 4, 1));
	comps.push_back(new LinearResistor("r_load", 4, 0, 1000));
	VillasInterface *villas = new VillasInterface("/villas1");
	villas->registerVoltageSource(evs, 0, 1);
	villas->registerExportedCurrent(evs, 0, 1);

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 0.3, log);
	newSim.addExternalInterface(villas);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, llog, rlog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
	log.WriteLogToFile("output.log");
	rlog.WriteLogToFile("rvector.log");
	llog.WriteLogToFile("lvector.log");
	for (auto comp : comps) {
		delete comp;
	}
	delete villas;
}
