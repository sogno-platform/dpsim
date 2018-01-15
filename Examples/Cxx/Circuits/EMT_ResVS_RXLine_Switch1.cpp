/** Reference Circuits
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

#include "Simulation.h"
#include "Utilities.h"

using namespace DPsim;
using namespace DPsim::Components::EMT;

int main(int argc, char* argv[])
{
	// Define simulation scenario
	Real timeStep = 0.001;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.3;
	String simName = "RxLineResLoad_EMT_" + std::to_string(timeStep);

	Components::Base::List comps0 = {
		VoltageSourceNorton::make("v_s", 0, GND, Complex(10000, 0), 1),
		Resistor::make("r_line", 0, 1, 1),
		Inductor::make("l_line", 1, 2, 1)
	};

	Components::Base::List comps1 = comps0;
	Components::Base::List comps2 = comps0;
	comps1.push_back(Resistor::make("r_load", 2, GND, 1000));
	comps2.push_back(Resistor::make("r_load", 2, GND, 800));

	Simulation sim(simName, comps1, omega, timeStep, finalTime, Logger::Level::INFO, SimulationType::EMT);
	sim.addSystemTopology(comps2);
	sim.setSwitchTime(0.1, 1);

	sim.run();

	return 0;
}
