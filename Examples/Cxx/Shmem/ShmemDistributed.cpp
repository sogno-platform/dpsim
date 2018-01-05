/** Example of shared memory interface
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

int main(int argc, char *argv[]) {
	Logger log;
	Component::Base::List comps, comps2;
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
		comps.push_back(std::make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
		comps.push_back(new Inductor("l_1", 1, 2, 0.1));
		comps.push_back(new LinearResistor("r_1", 2, 3, 1));
		ExternalVoltageSource *evs = new ExternalVoltageSource("v_t", 3, 0, Complex(0, 0), 1);
		comps.push_back(evs);
		shmem = new ShmemInterface("/villas1-in", "/villas1-out", &conf);
		shmem->registerVoltageSource(evs, 0, 1);
		shmem->registerExportedCurrent(evs, 0, 1);
	}
	else if (!strcmp(argv[1], "1")) {
		logname = "lvector1.log";
		ExternalCurrentSource *ecs = new ExternalCurrentSource("v_s", 1, 0, Complex(0, 0));
		comps.push_back(ecs);
		comps.push_back(new LinearResistor("r_2", 1, 0, 10));
		shmem = new ShmemInterface("/villas2-in", "/villas2-out", &conf);
		shmem->registerCurrentSource(ecs, 0, 1);
		shmem->registerExportedVoltage(1, 0, 0, 1);
	}
	else {
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

	for (auto comp : comps)
		delete comp;

	delete shmem;
}
