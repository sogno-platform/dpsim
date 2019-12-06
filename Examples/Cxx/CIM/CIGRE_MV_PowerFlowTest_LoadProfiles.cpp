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
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){

	#ifdef _WIN32
		String loadProfilePath("..\\..\\..\\..\\..\\sogno-grid-data-public\\Load_Data\\CIGRE_MV_NoTap\\");
	#elif defined(__linux__) || defined(__APPLE__)
		String loadProfilePath("../sogno-grid-data-public/Load_Data/CIGRE_MV_NoTap/");
	#endif
	
	std::map<String,String> assignList = {
	// {load mRID, file name}
	{"LOAD-H-1", "Load_H_1"},
	{"LOAD-H-3", "Load_H_3"},
	{"LOAD-H-4", "Load_H_4"},
	{"LOAD-H-5", "Load_H_5"},
	{"LOAD-H-6", "Load_H_6"},
	{"LOAD-H-8", "Load_H_8"},
	{"LOAD-H-10", "Load_H_10"},
	{"LOAD-H-11", "Load_H_11"},
	{"LOAD-H-12", "Load_H_12"},
	{"LOAD-H-14", "Load_H_14"},
	{"LOAD-I-1", "Load_I_1"},
	{"LOAD-I-3", "Load_I_3"},
	{"LOAD-I-7", "Load_I_7"},
	{"LOAD-I-9", "Load_I_9"},
	{"LOAD-I-10", "Load_I_10"},
	{"LOAD-I-12", "Load_I_12"},
	{"LOAD-I-13", "Load_I_13"},
	{"LOAD-I-14", "Load_I_14"}};

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_06J16h_DI.xml",
			"Rootnet_FULL_NE_06J16h_EQ.xml",
			"Rootnet_FULL_NE_06J16h_SV.xml",
			"Rootnet_FULL_NE_06J16h_TP.xml"
		}, "Examples/CIM/CIGRE_MV_NoTap", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	String simName = "CIGRE-MV-NoTap-LoadProfiles";
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::info, Logger::Level::off);
    SystemTopology system = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);
	
	CSVReader csvreader(simName, loadProfilePath, assignList, Logger::Level::info);
	csvreader.assignLoadProfile(system, 0, 1.5, 15, CSVReader::Mode::MANUAL);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : system.mNodes)
	{
		logger->addAttribute(node->name(), node->attribute("v"));
		std::list<std::shared_ptr<CPS::SP::Ph1::PiLine>> lines;
		for (auto comp : system.mComponentsAtNode[node]) {
			if (std::shared_ptr<CPS::SP::Ph1::PiLine> line =
				std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp))
			{
				String current=(node->name() == line->node(0)->name()) ? ("current") : ("current_1");
				String p_branch = (node->name() == line->node(0)->name()) ? ("p_branch") : ("p_branch_1");
				String q_branch = (node->name() == line->node(0)->name()) ? ("q_branch") : ("q_branch_1");

				logger->addAttribute(line->name() + "." + node->name() + ".I", line->attribute<Complex>(current));
				logger->addAttribute(line->name() + "." + node->name() + ".P", line->attribute<Real>(p_branch));
				logger->addAttribute(line->name() + "." + node->name() + ".Q", line->attribute<Real>(q_branch));
				lines.push_back(line);
			}
			else if (std::shared_ptr<CPS::SP::Ph1::externalGridInjection> extnet =
				std::dynamic_pointer_cast<CPS::SP::Ph1::externalGridInjection>(comp))
			{
				logger->addAttribute(node->name() + ".Pinj", extnet->attribute<Real>("p_inj"));
				logger->addAttribute(node->name() + ".Qinj", extnet->attribute<Real>("q_inj"));
			}
			else if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo =
				std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp))
			{
				String current = (node->name() == trafo->node(0)->name()) ? ("current") : ("current_1");
				String p_branch = (node->name() == trafo->node(0)->name()) ? ("p_branch") : ("p_branch_1");
				String q_branch = (node->name() == trafo->node(0)->name()) ? ("q_branch") : ("q_branch_1");

				logger->addAttribute(trafo->name() + "." + node->name() + ".I", trafo->attribute<Complex>(current));
				logger->addAttribute(trafo->name() + "." + node->name() + ".P", trafo->attribute<Real>(p_branch));
				logger->addAttribute(trafo->name() + "." + node->name() + ".Q", trafo->attribute<Real>(q_branch));

			}
		}
		// get nodal injection from specific line or transformer
		// (the first line obj connected to the node or, if none, the first trafo)
		if (!lines.empty()) {
		logger->addAttribute(node->name() + ".Pinj", lines.front()->attribute<Real>("p_inj"));
		logger->addAttribute(node->name() + ".Qinj", lines.front()->attribute<Real>("q_inj"));
		}
		else
		{
			for (auto comp : system.mComponentsAtNode[node]) {
				if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo =
					std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp))
				{
					logger->addAttribute(node->name() + ".Pinj", trafo->attribute<Real>("p_inj"));
					logger->addAttribute(node->name() + ".Qinj", trafo->attribute<Real>("q_inj"));
					break;
				}

			}
		}
	}

	Simulation sim(simName, system, 1.5, 15, Domain::SP, Solver::Type::NRP, Logger::Level::info, true);

	sim.addLogger(logger);
	sim.run();

	return 0;
}

