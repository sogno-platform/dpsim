/**
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
*
* CPowerSystems
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

#include <cps/DP/DP_Ph1_Inverter.h>

using namespace CPS;
using namespace CPS::DP;

int main(int argc, char *argv[]) {
	String simName = "Inverter_test";
	Logger::setLogDir("logs/"+simName);

	auto inv = Ph1::Inverter::make("inv", Logger::Level::debug);
	inv->setParameters( std::vector<CPS::Int>{2,2,2,2,4,4,4,4},
						std::vector<CPS::Int>{-3,-1,1,3,-5,-1,1,5},
						360, 0.87, 0);
	//inv->initialize(50, );
	inv->calculatePhasors();
}
