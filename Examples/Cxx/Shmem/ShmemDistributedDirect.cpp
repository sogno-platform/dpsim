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

int main(int argc, char *argv[])
{
	// Testing the interface with a simple circuit,
	// but the load is simulated in a different instance.
	// Values are exchanged using the ideal transformator model: an ideal
	// current source on the supply side and an ideal voltage source on the
	// supply side, whose values are received from the respective other circuit.
	// Here, the two instances directly communicate with each other without using
	// VILLASnode in between.
	Logger log;
	Component::Base::List comps;
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
		comps.push_back(std::make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
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

	for (auto comp : comps)
		delete comp;

	delete shmem;
}
