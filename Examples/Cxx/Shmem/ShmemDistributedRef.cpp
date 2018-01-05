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

int main(int argc, char* argv[])
{
	// Same circuit as above, but the simulation is done normally in one instance.
	Logger log("output.log"), llog("lvector.log"), rlog("rvector.log");
	Component::Base::List comps, comps2;

	comps.push_back(std::make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
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

	for (auto comp : comps)
		delete comp;

	return 0;
}
