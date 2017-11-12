/** Shared-memory interface Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include "ShmemTest.h"
#include "Simulation.h"
#include "ShmemInterface.h"
#include "Utilities.h"

using namespace DPsim;

void DPsim::shmemExample()
{
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.
	Logger log("output.log"), llog("lvector.log"), rlog("rvector.log");
	ElementList comps;

	ExternalVoltageSource *evs = new ExternalVoltageSource("v_s", 1, 0, Complex(0, 0), 1);
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
	ElementList comps;
	struct shmem_conf conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = false;
	Logger log;

	ExternalVoltageSource *evs = new ExternalVoltageSource("v_s", 1, 0, Complex(0, 0), 1);
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
	newSim.runRT(RTExceptions, false, log, log, log);
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
	ElementList comps;
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
		comps.push_back(make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
		comps.push_back(new Inductor("l_1", 1, 2, 1e-3));
		ExternalVoltageSource *evs = new ExternalVoltageSource("v_t", 2, 0, Complex(0, 0), 1);
		comps.push_back(evs);
		shmem = new ShmemInterface("/dpsim01", "/dpsim10", &conf);
		shmem->registerVoltageSource(evs, 0, 1);
		shmem->registerExportedCurrent(evs, 0, 1);
	} else if (!strcmp(argv[1], "1")) {
		ExternalCurrentSource *ecs = new ExternalCurrentSource("v_s", 1, 0, Complex(0, 0));
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
	newSim.runRT(RTTimerFD, false, log, log, log);
	std::cout << "Simulation finished." << std::endl;

	for (auto comp : comps) {
		delete comp;
	}
	delete shmem;
}

void DPsim::shmemDistributed(int argc, char *argv[])
{
	Logger log;
	ElementList comps, comps2;
	ShmemInterface *shmem;
	struct shmem_conf conf;
	String logname;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = true;

	if (argc < 2) {
		std::cerr << "not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	if (!strcmp(argv[1], "0")) {
		logname = "lvector0.log";
		comps.push_back(make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
		comps.push_back(new Inductor("l_1", 1, 2, 0.1));
		comps.push_back(new LinearResistor("r_1", 2, 3, 1));
		ExternalVoltageSource *evs = new ExternalVoltageSource("v_t", 3, 0, Complex(0, 0), 1);
		comps.push_back(evs);
		shmem = new ShmemInterface("/villas1-in", "/villas1-out", &conf);
		shmem->registerVoltageSource(evs, 0, 1);
		shmem->registerExportedCurrent(evs, 0, 1);
	} else if (!strcmp(argv[1], "1")) {
		logname = "lvector1.log";
		ExternalCurrentSource *ecs = new ExternalCurrentSource("v_s", 1, 0, Complex(0, 0));
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
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 20, log);
	newSim.addExternalInterface(shmem);
	if (!strcmp(argv[1], "1")) {
		comps2 = comps;
		comps2.pop_back();
		comps2.push_back(new LinearResistor("r_2", 1, 0, 8));
		newSim.addSystemTopology(comps2);
		newSim.setSwitchTime(10, 1);
	}

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	newSim.runRT(RTTimerFD, true, log, llog, log);
	std::cout << "Simulation finished." << std::endl;

	if (!strcmp(argv[1], "1"))
		delete comps2.back();
	for (auto comp : comps) {
		delete comp;
	}
	delete shmem;
}

void DPsim::shmemDistributedRef()
{
	// Same circuit as above, but the simulation is done normally in one instance.
	Logger log("output.log"), llog("lvector.log"), rlog("rvector.log");
	ElementList comps, comps2;

	comps.push_back(make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
	comps.push_back(new Inductor("l_1", 1, 2, 0.1));
	comps.push_back(new LinearResistor("r_1", 2, 3, 1));
	comps2 = comps;
	comps.push_back(new LinearResistor("r_2", 3, 0, 10));
	comps2.push_back(new LinearResistor("r_2", 3, 0, 8));

	// Set up simulation
	Real timeStep = 0.001;
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 20, log);
	newSim.addSystemTopology(comps2);
	newSim.setSwitchTime(10, 1);

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

void DPsim::exampleMainShmem() {
	// TODO: RT / shmem interface with python

	ShmemInterface *intf = nullptr;
	// TODO: this is a simple, pretty much fixed setup. Make this more flexible / configurable
	if (split >= 0) {
		int node = reader.mapTopologicalNode(splitNode);
		if (node < 0) {
			std::cerr << "Invalid / missing split node" << std::endl;
			return 1;
		}

		if (split == 0) {
			outName = interfaceBase + ".0.out";
			inName = interfaceBase + ".0.in";
			intf = new ShmemInterface(outName.c_str(), inName.c_str());
			ExternalVoltageSource *evs = new ExternalVoltageSource("v_int", node, 0, 0, 0, reader.getNumVoltageSources() + 1);
			intf->registerVoltageSource(evs, 0, 1);
			intf->registerExportedCurrent(evs, 0, 1);
			components.push_back(evs);
			// TODO make log names configurable
			logName = "cim0.log";
			llogName = "lvector-cim0.csv";
			rlogName = "rvector-cim0.csv";
		}
		else {
			outName = interfaceBase + ".1.out";
			inName = interfaceBase + ".1.in";
			intf = new ShmemInterface(outName.c_str(), inName.c_str());
			ExternalCurrentSource *ecs = new ExternalCurrentSource("i_int", node, 0, 0, 0);
			intf->registerCurrentSource(ecs, 0, 1);
			intf->registerExportedVoltage(node, 0, 0, 1);
			components.push_back(ecs);
			logName = "cim1.log";
			llogName = "lvector-cim1.csv";
			rlogName = "rvector-cim1.csv";
		}
	}

	if (intf) {
		sim.addExternalInterface(intf);
	}

	if (rt) {
		sim.runRT(RTTimerFD, true, log, llog, rlog);
	} 
	else {
		while (sim.step(log, llog, rlog)) {
			sim.increaseByTimeStep();
		}
	}

	if (intf) {
		delete intf;
	}
}
