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

#include "DPsim.h"

using namespace DPsim;
using namespace CPS::Components::DP;

int main(int argc, char* argv[]) {	
	// Define system topology
	SystemTopology system(50, {
		VoltageSource::make("v_1", 0, GND, Complex(10, 0)),
		RxLine::make("Line_1", 0, 1, 0.1, 0.001, RxLine::Node3),
		Resistor::make("r_1", 1, GND, 20)});

	// Define simulation scenario
	Real timeStep = 0.00001;
	Real finalTime = 0.1;
	String simName = "DP_IdealVS_RxLine1_" + std::to_string(timeStep);

	Simulation sim(simName, system, timeStep, finalTime);
	sim.run();

	return 0;
}
