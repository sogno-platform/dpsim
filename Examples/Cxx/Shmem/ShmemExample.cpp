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
using namespace DPsim::Components::DP;

int main(int argc, char* argv[])
{
	// Very simple test circuit. Just a few resistors and an inductance.
	// Voltage is read from VILLASnode and current through everything is written back.

	auto evs = VoltageSource::make("v_s", 1, 0, Complex(0, 0));

	Component::List comps = {
		evs,
		Resistor::make("r_s", 0, 1, 1),
		Resistor::make("r_line", 1, 2, 1),
		Inductor::make("l_line", 2, 3, 1),
		Resistor::make("r_load", 3, GND, 1000)
	};

	ShmemInterface villas("/villas1-in", "/villas1-out");
	villas.registerControllableSource(evs, GND, 0);
	villas.registerExportedCurrent(evs, GND, 0);

	Real timeStep = 0.001;
	Simulation sim("ShmemExample", comps, 2.0*M_PI*50.0, timeStep, 0.3, Logger::Level::INFO);
	sim.addExternalInterface(&villas);

	sim.run();

	return 0;
}
