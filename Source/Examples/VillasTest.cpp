#include "VillasTest.h"
#include "../Simulation.h"
#include "../VillasInterface.h"
#include "../Utilities.h"

using namespace DPsim;

void DPsim::villasExample()
{
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.
	Logger log, llog, rlog;
	std::vector<BaseComponent*> comps;

	ExternalVoltageSource *evs = new ExternalVoltageSource("v_s", 1, 0, 0, 0, 1);
	comps.push_back(evs);
	comps.push_back(new LinearResistor("r_s", 1, 2, 1));
	comps.push_back(new LinearResistor("r_line", 2, 3, 1));
	comps.push_back(new Inductor("l_line", 3, 4, 1));
	comps.push_back(new LinearResistor("r_load", 4, 0, 1000));
	VillasInterface *villas = new VillasInterface("/villas1-in", "/villas1-out");
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

void DPsim::villasDistributedExample(int argc, char *argv[])
{
	// Testing the interface with a simple circuit,
	// but the load is simulated in a different instance.
	// Values are exchanged using the ideal transformator model: an ideal
	// current source on the supply side and an ideal voltage source on the
	// supply side, whose values are received from the respective other circuit.
	Logger log, llog, rlog;
	std::vector<BaseComponent*> comps;
	VillasInterface *villas;

	if (argc < 2) {
		std::cerr << "not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	if (!strcmp(argv[1], "0")) {
		comps.push_back(new VoltSourceRes("v_s", 1, 0, 10000, 0, 1));
		comps.push_back(new Inductor("l_1", 1, 2, 1e-3));
		ExternalVoltageSource *evs = new ExternalVoltageSource("v_t", 2, 0, 0, 0, 1);
		comps.push_back(evs);
		villas = new VillasInterface("/dpsim01", "/dpsim10");
		villas->registerVoltageSource(evs, 0, 1);
		villas->registerExportedCurrent(evs, 0, 1);
	} else if (!strcmp(argv[1], "1")) {
		ExternalCurrentSource *ecs = new ExternalCurrentSource("v_s", 1, 0, 0, 0);
		comps.push_back(ecs);
		comps.push_back(new LinearResistor("r_2", 1, 0, 1));
		villas = new VillasInterface("/dpsim10", "/dpsim01");
		villas->registerCurrentSource(ecs, 0, 1);
		villas->registerExportedVoltage(1, 0, 0, 1);
	} else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

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
	log.WriteLogToFile("output" + std::string(argv[1]) + ".log");
	rlog.WriteLogToFile("rvector" + std::string(argv[1]) + ".log");
	llog.WriteLogToFile("lvector" + std::string(argv[1]) + ".log");
	for (auto comp : comps) {
		delete comp;
	}
	delete villas;
}

void DPsim::villasDistributedRef()
{
	// Same circuit as above, but the simulation is done normally in one instance.
	Logger log, llog, rlog;
	std::vector<BaseComponent*> comps;

	comps.push_back(new VoltSourceRes("v_s", 1, 0, 10000, 0, 1));
	comps.push_back(new Inductor("l_1", 1, 2, 1e-3));
	comps.push_back(new LinearResistor("r_2", 2, 0, 1));

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 0.3, log);

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
}
