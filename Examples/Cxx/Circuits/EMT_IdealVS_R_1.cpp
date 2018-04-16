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
using namespace CPS::Components::EMT;

int main(int argc, char* argv[]) {
	// Define system topology
	SystemTopology system(50, {
		VoltageSource::make("v_in", 0, 1, 10, 0),
		Resistor::make("r_1", 0, GND, 5),
		Resistor::make("r_2", 1, GND, 10),
		Resistor::make("r_3", 1, GND, 2)});

	// Define simulation scenario
	Real timeStep = 0.00005;
	Real finalTime = 0.2;
	String simName = "EMT_IdealVS_R1";

	Simulation sim(simName, system, timeStep, finalTime, Solver::Domain::EMT);
	sim.run();
	
	return 0;
}
