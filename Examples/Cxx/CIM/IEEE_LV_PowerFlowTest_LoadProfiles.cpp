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
		}, "Examples/CIM/IEEE_EU_LV_reduced", "CIMPATH");
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

