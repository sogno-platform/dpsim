/** CIM Test
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

#include <iostream>
#include <list>

#include "DPsim.h"

using namespace DPsim;

int main(int argc, char *argv[]) {
#ifdef _WIN32
	String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\WSCC-09_Neplan_RX\\");
#elif defined(__linux__)
	String path("../../../dpsim/Examples/CIM/IEEE-09_Neplan_RX/");
#endif
	
	std::list<String> filenames = {
		path + "WSCC-09_Neplan_RX_DI.xml",
		path + "WSCC-09_Neplan_RX_EQ.xml",
		path + "WSCC-09_Neplan_RX_SV.xml",
		path + "WSCC-09_Neplan_RX_TP.xml"
	};

	Simulation sim("CIM", filenames, 50, 0.0001, 0.1,
		Solver::Domain::DP, Solver::Type::MNA, Logger::Level::DEBUG);
	sim.run();

	return 0;
}
