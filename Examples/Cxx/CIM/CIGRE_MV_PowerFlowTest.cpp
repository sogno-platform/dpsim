/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "cps/CIM/Reader.h"
#include <DPsim.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;


/*
 * Power flow test script
 */
int main(int argc, char** argv){

	string path("D:\\git\\data\\cim-grid-data\\CIGRE_MV\\CIGRE_MV_no_tapchanger_With_LoadFlow_Results\\");
	/* CIGRE-MV-NoTap 

	file path on jzh
	string path("D:\\HiWi_ACS\\grid_data\\cim-grid-data\\CIGRE_MV\\CIGRE_MV_no_tapchanger_With_LoadFlow_Results\\");
	*/
	std::list<string> filenames = {
	path + "Rootnet_FULL_NE_06J16h_DI.xml",
	path + "Rootnet_FULL_NE_06J16h_EQ.xml",
	path + "Rootnet_FULL_NE_06J16h_SV.xml",
	path + "Rootnet_FULL_NE_06J16h_TP.xml"
	};
	String simName = "CIGRE-MV-NoTap-Neplan";
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::DEBUG, Logger::Level::NONE);
    SystemTopology system = reader.loadCIM(system_freq, filenames, CPS::Domain::Static);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : system.mNodes)
	{
		logger->addAttribute(node->name(), node->attribute("v"));
	}
	Simulation sim(simName, system, 0.001, 0.1, Domain::Static, Solver::Type::NRP, Logger::Level::DEBUG, true);

	sim.addLogger(logger);
	sim.run();



	return 0;
}

