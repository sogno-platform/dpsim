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

void main(int argc, char* argv[])
{
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.
	Logger log("output.log"), llog("lvector.log"), rlog("rvector.log");
	Components::Base::List comps;

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

	while (newSim.step(log, llog, rlog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}

	std::cout << "Simulation finished." << std::endl;

	for (auto comp : comps)
		delete comp;

	delete villas;
}
