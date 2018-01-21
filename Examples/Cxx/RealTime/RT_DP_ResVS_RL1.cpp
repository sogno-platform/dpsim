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

#include "RealTimeSimulation.h"
#include "Components.h"

using namespace DPsim;
using namespace DPsim::Components::DP;

int main(int argc, char* argv[])
{
	Component::List comps = {
		VoltageSource::make("v_s", 0, GND, Complex(10000, 0)),
		Resistor::make("r_line", 0, 1, 1),
		Inductor::make("l_line", 1, 2, 1),
		Resistor::make("r_load", 2, GND, 1000)
	};

	Real timeStep = 0.00005;
	RealTimeSimulation sim("RT_DP_ResVS_RL1", comps, 2.0*M_PI*50.0, timeStep, 1.0);

	sim.run();

	return 0;
}
