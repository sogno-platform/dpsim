/**
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2019, Institute for Automation of Complex Power Systems, EONERC
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

#include "cps/CIM/Reader.h"
#include <DPsim.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;


/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){

	#ifdef _WIN32
		String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\CIGRE_MV_NoTap\\");
	#elif defined(__linux__) || defined(__APPLE__)
		String path("Examples/CIM/CIGRE_MV_NoTap/");
	#endif

	std::list<string> filenames = {
	path + "Rootnet_FULL_NE_06J16h_DI.xml",
	path + "Rootnet_FULL_NE_06J16h_EQ.xml",
	path + "Rootnet_FULL_NE_06J16h_SV.xml",
	path + "Rootnet_FULL_NE_06J16h_TP.xml"
	};
	String simName = "CIGRE-MV-NoTap";
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::DEBUG, Logger::Level::NONE);
    SystemTopology system = reader.loadCIM(system_freq, filenames, CPS::Domain::Static);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : system.mNodes)
	{
		logger->addAttribute(node->name(), node->attribute("v"));
	}

	Simulation sim(simName, system, 1, 60, Domain::Static, Solver::Type::NRP, Logger::Level::DEBUG, true);

	sim.addLogger(logger);
	sim.run();

	return 0;
}

