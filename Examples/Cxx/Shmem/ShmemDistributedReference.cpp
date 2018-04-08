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

#include "DPsim.h"

using namespace DPsim;
using namespace CPS::Components::DP;

int main(int argc, char* argv[])
{
	// Same circuit as above, but the simulation is done normally in one instance.
	Component::List comps = {
		VoltageSourceNorton::make("v_s", 0, GND, Complex(10000, 0), 1),
		Inductor::make("l_1", 0, 1, 0.1),
		Resistor::make("r_1", 1, 2, 1)
	};

	auto comps2 = comps;
	comps.push_back(Resistor::make("r_2", 2, GND, 10));
	comps2.push_back(Resistor::make("r_2", 2, GND, 8));

	Real timeStep = 0.001;
	Simulation sim("ShmemDistirbutedRef", comps, 2.0*M_PI*50.0, timeStep, 20, Logger::Level::INFO);
	sim.addSystemTopology(comps2);
	sim.setSwitchTime(10, 1);
	sim.run();

	return 0;
}
