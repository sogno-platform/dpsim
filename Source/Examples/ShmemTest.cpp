#include "ShmemTest.h"
#include "../Simulation.h"
#include "../ShmemInterface.h"
#include "../Utilities.h"

using namespace DPsim;

void DPsim::shmemExample()
{
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.
	Logger log("output.log"), llog("lvector.log"), rlog("rvector.log");
	std::vector<BaseComponent*> comps;

	ExternalVoltageSource *evs = new ExternalVoltageSource("v_s", 1, 0, 0, 0, 1);
	comps.push_back(evs);
	comps.push_back(new LinearResistor("r_s", 1, 2, 1));
	comps.push_back(new LinearResistor("r_line", 2, 3, 1));
	comps.push_back(new Inductor("l_line", 3, 4, 1));
	comps.push_back(new LinearResistor("r_load", 4, 0, 1000));
	ShmemInterface *villas = new ShmemInterface("/villas1-in", "/villas1-out");
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
	for (auto comp : comps) {
		delete comp;
	}
	delete villas;
}

void DPsim::shmemRTExample()
{
	// Same circuit as above, but now with realtime support.
	std::vector<BaseComponent*> comps;
	struct shmem_conf conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = false;
	Logger log;

	ExternalVoltageSource *evs = new ExternalVoltageSource("v_s", 1, 0, 0, 0, 1);
	comps.push_back(evs);
	comps.push_back(new LinearResistor("r_s", 1, 2, 1));
	comps.push_back(new LinearResistor("r_line", 2, 3, 1));
	comps.push_back(new Inductor("l_line", 3, 4, 1));
	comps.push_back(new LinearResistor("r_load", 4, 0, 1000));
	ShmemInterface *villas = new ShmemInterface("/villas1-in", "/villas1-out", &conf);
	villas->registerVoltageSource(evs, 0, 1);
	villas->registerExportedCurrent(evs, 0, 1);

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 5.0, log);
	newSim.addExternalInterface(villas);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	newSim.runRTSignal(log);
	std::cout << "Simulation finished." << std::endl;
	for (auto comp : comps) {
		delete comp;
	}
	delete villas;
}

void DPsim::shmemDistributedDirect(int argc, char *argv[])
{
	// Testing the interface with a simple circuit,
	// but the load is simulated in a different instance.
	// Values are exchanged using the ideal transformator model: an ideal
	// current source on the supply side and an ideal voltage source on the
	// supply side, whose values are received from the respective other circuit.
	// Here, the two instances directly communicate with each other without using
	// VILLASnode in between.
	Logger log;
	std::vector<BaseComponent*> comps;
	ShmemInterface *shmem;
	struct shmem_conf conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = false;

	if (argc < 2) {
		std::cerr << "not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	if (!strcmp(argv[1], "0")) {
		comps.push_back(new VoltSourceRes("v_s", 1, 0, 10000, 0, 1));
		comps.push_back(new Inductor("l_1", 1, 2, 1e-3));
		ExternalVoltageSource *evs = new ExternalVoltageSource("v_t", 2, 0, 0, 0, 1);
		comps.push_back(evs);
		shmem = new ShmemInterface("/dpsim01", "/dpsim10", &conf);
		shmem->registerVoltageSource(evs, 0, 1);
		shmem->registerExportedCurrent(evs, 0, 1);
	} else if (!strcmp(argv[1], "1")) {
		ExternalCurrentSource *ecs = new ExternalCurrentSource("v_s", 1, 0, 0, 0);
		comps.push_back(ecs);
		comps.push_back(new LinearResistor("r_2", 1, 0, 1));
		shmem = new ShmemInterface("/dpsim10", "/dpsim01", &conf);
		shmem->registerCurrentSource(ecs, 0, 1);
		shmem->registerExportedVoltage(1, 0, 0, 1);
	} else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

	// Set up simulation
	Real timeStep = 0.000150;
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 1, log);
	newSim.addExternalInterface(shmem);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	newSim.runRTTimerfd(log, log, log);
	std::cout << "Simulation finished." << std::endl;

	for (auto comp : comps) {
		delete comp;
	}
	delete shmem;
}

void DPsim::shmemDistributed(int argc, char *argv[])
{
	Logger log;
	std::vector<BaseComponent*> comps;
	ShmemInterface *shmem;
	struct shmem_conf conf;
	std::string logname;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = true;

	if (argc < 2) {
		std::cerr << "not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	if (!strcmp(argv[1], "0")) {
		logname = "lvector0.log";
		comps.push_back(new VoltSourceRes("v_s", 1, 0, 10000, 0, 1));
		comps.push_back(new Inductor("l_1", 1, 2, 0.1));
		comps.push_back(new LinearResistor("r_1", 2, 3, 1));
		ExternalVoltageSource *evs = new ExternalVoltageSource("v_t", 3, 0, 0, 0, 1);
		comps.push_back(evs);
		shmem = new ShmemInterface("/villas1-in", "/villas1-out", &conf);
		shmem->registerVoltageSource(evs, 0, 1);
		shmem->registerExportedCurrent(evs, 0, 1);
	} else if (!strcmp(argv[1], "1")) {
		logname = "lvector1.log";
		ExternalCurrentSource *ecs = new ExternalCurrentSource("v_s", 1, 0, 0, 0);
		comps.push_back(ecs);
		comps.push_back(new LinearResistor("r_2", 1, 0, 10));
		shmem = new ShmemInterface("/villas2-in", "/villas2-out", &conf);
		shmem->registerCurrentSource(ecs, 0, 1);
		shmem->registerExportedVoltage(1, 0, 0, 1);
	} else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

	// Set up simulation
	Real timeStep = 0.001000;
	Logger llog(logname);
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 1, log);
	newSim.addExternalInterface(shmem);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	newSim.runRTTimerfd(log, llog, log);
	std::cout << "Simulation finished." << std::endl;

	for (auto comp : comps) {
		delete comp;
	}
	delete shmem;
}

void DPsim::shmemDistributedRef()
{
	// Same circuit as above, but the simulation is done normally in one instance.
	Logger log("output.log"), llog("lvector.log"), rlog("rvector.log");
	std::vector<BaseComponent*> comps;

	comps.push_back(new VoltSourceRes("v_s", 1, 0, 10000, 0, 1));
	comps.push_back(new Inductor("l_1", 1, 2, 0.1));
	comps.push_back(new LinearResistor("r_1", 2, 3, 1));
	comps.push_back(new LinearResistor("r_2", 3, 0, 10));

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 1, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, llog, rlog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto comp : comps) {
		delete comp;
	}
}
