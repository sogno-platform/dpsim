/** Reference Circuits
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

int main() {
	Real timeStep = 0.00005;
	Logger log;
	ElementList comps;
	comps.push_back(std::make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
	comps.push_back(std::make_shared<ResistorDP>("r_line", 1, 2, 1));
	comps.push_back(std::make_shared<InductorDP>("l_line", 2, 3, 1));
	comps.push_back(std::make_shared<ResistorDP>("r_load", 3, 0, 1000));

	// Set up simulation
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 1.0, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;

	newSim.runRT(RTExceptions, false, log, log, log);

	std::cout << "Simulation finished." << std::endl;

	return 0;
}
