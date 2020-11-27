/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/CIM/Reader.h>
#include <DPsim.h>
#include <cps/CSVReader.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;


/*
 * This example runs the powerflow for the IEEE LV EU benchmark system
 */

int main(int argc, char** argv){

	#ifdef _WIN32
			String loadProfilePath("..\\..\\..\\..\\SimulationData\\Load\\IEEE_European_LV_TestFeeder_v2_Profiles\\csv\\");
	#elif defined(__linux__) || defined(__APPLE__)
			String loadProfilePath("../SimulationData/Load/IEEE_European_LV_TestFeeder_v2_Profiles/csv/");
	#endif

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_06J16h_DI.xml",
			"Rootnet_FULL_NE_06J16h_EQ.xml",
			"Rootnet_FULL_NE_06J16h_SV.xml",
			"Rootnet_FULL_NE_06J16h_TP.xml"
		}, "build/_deps/cim-data-src/IEEE_EU_LV/IEEE_EU_LV_reduced", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "IEEE-LV-reduced";
	CPS::Real system_freq = 50;

	/*
		create assign list
	*/
	std::map<String, String> assignList;
	for (const auto & entry : std::experimental::filesystem::directory_iterator(loadProfilePath))
	{
		string filename = entry.path().filename().string();
		string load_number;
		for (auto &c : filename) {
			if (isdigit(c)) {
				load_number += c;
			}
		}
		assignList.insert(std::pair<string,string>("PQ"+load_number,"Load_profile_"+load_number));
	}

    CIM::Reader reader(simName, Logger::Level::info, Logger::Level::off);
    SystemTopology system = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	//load profile lpreader
	CSVReader csvreader(simName, loadProfilePath, assignList, Logger::Level::info);
	csvreader.assignLoadProfile(system, 1, 1, 30, CSVReader::Mode::MANUAL, CSVReader::DataFormat::HHMMSS);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : system.mNodes)
	{
		logger->addAttribute(node->name(), node->attribute("v"));
	}

	Simulation sim(simName, system, 1, 30, Domain::SP, Solver::Type::NRP, Logger::Level::info, true);

	sim.addLogger(logger);
	sim.run();

	return 0;
}

