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

int main(int argc, char* argv[])
{
	// Define simulation scenario
	Real timeStep = 0.001;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.3;
	String simName = "IdealVS1_" + std::to_string(timeStep);

	Components::Base::List comps = {
		std::make_shared<Components::DP::VoltageSourceIdeal>("v_in", 1, 2, Complex(10, 0)),
		std::make_shared<Components::DP::Resistor>("r_1", 1, 0, 1),
		std::make_shared<Components::DP::Resistor>("r_2", 2, 0, 1),
		std::make_shared<Components::DP::Resistor>("r_3", 2, 0, 1)
	};

	// Set up simulation and start main simulation loop
	Simulation newSim(simName, comps, omega, timeStep, finalTime);

	std::cout << "Start simulation." << std::endl;
	newSim.run();
	std::cout << "Simulation finished." << std::endl;

	return 0;
}
