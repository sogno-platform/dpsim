/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/CIM/Reader.h>
#include <DPsim.h>
#include <dpsim-models/CSVReader.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;


/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){

	#ifdef _WIN32
		String loadProfilePath("build\\_deps\\profile-data-src\\CIGRE_MV_NoTap\\load_profiles\\");
	#elif defined(__linux__) || defined(__APPLE__)
		String loadProfilePath("build/_deps/profile-data-src/CIGRE_MV_NoTap/load_profiles/");
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
	filenames = DPsim::Utils::findFiles({
		"Rootnet_FULL_NE_06J16h_DI.xml",
		"Rootnet_FULL_NE_06J16h_EQ.xml",
		"Rootnet_FULL_NE_06J16h_SV.xml",
		"Rootnet_FULL_NE_06J16h_TP.xml"
	}, "build/_deps/cim-data-src/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_With_LoadFlow_Results/", "CIMPATH");

	String simName = "CIGRE-MV-NoTap-LoadProfiles";
	CPS::Real system_freq = 50;

	CPS::Real time_begin = 0;
	CPS::Real time_step = 1;
	CPS::Real time_end = 300;

	if (argc > 1) {
		CommandLineArgs args(argc, argv);
		time_step = args.timeStep;
		time_end = args.duration;
	}

    CIM::Reader reader(Logger::Level::info, Logger::Level::off, Logger::Level::off);
    SystemTopology system = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	CSVReader csvreader(loadProfilePath, assignList, Logger::Level::info);
	csvreader.assignLoadProfile(system, time_begin, time_step, time_end, CSVReader::Mode::MANUAL);

	auto logger = DPsim::DataLogger::make(simName);
	for (auto node : system.mNodes)
	{
		logger->logAttribute(node->name(), node->attribute("v"));
		std::list<std::shared_ptr<CPS::SP::Ph1::PiLine>> lines;
		for (auto comp : system.mComponentsAtNode[node]) {
			if (std::shared_ptr<CPS::SP::Ph1::PiLine> line =
				std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp))
			{
				Bool isNodeZero = node->name() == line->node(0)->name();
				logger->logAttribute(line->name() + "." + node->name() + ".I", line->mCurrent->deriveCoeff<Complex>(isNodeZero ? 0 : 1, 0));
				logger->logAttribute(line->name() + "." + node->name() + ".P", line->mActivePowerBranch->deriveCoeff<Real>(isNodeZero ? 0 : 1, 0));
				logger->logAttribute(line->name() + "." + node->name() + ".Q", line->mReactivePowerBranch->deriveCoeff<Real>(isNodeZero ? 0 : 1, 0));

				lines.push_back(line);
			}
			else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet =
				std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp))
			{
				logger->logAttribute(node->name() + ".Pinj", extnet->mActivePowerInjection);
				logger->logAttribute(node->name() + ".Qinj", extnet->mReactivePowerInjection);
			}
			else if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo =
				std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp))
			{
				Bool isNodeZero = node->name() == trafo->node(0)->name();
				logger->logAttribute(trafo->name() + "." + node->name() + ".I", trafo->mCurrent->deriveCoeff<Complex>(isNodeZero ? 0 : 1, 0));
				logger->logAttribute(trafo->name() + "." + node->name() + ".P", trafo->mActivePowerBranch->deriveCoeff<Real>(isNodeZero ? 0 : 1, 0));
				logger->logAttribute(trafo->name() + "." + node->name() + ".Q", trafo->mReactivePowerBranch->deriveCoeff<Real>(isNodeZero ? 0 : 1, 0));
			}
		}
		// get nodal injection from specific line or transformer
		// (the first line obj connected to the node or, if none, the first trafo)
		if (!lines.empty()) {
		logger->logAttribute(node->name() + ".Pinj", lines.front()->attributeTyped<Real>("p_inj"));
		logger->logAttribute(node->name() + ".Qinj", lines.front()->attributeTyped<Real>("q_inj"));
		}
		else
		{
			for (auto comp : system.mComponentsAtNode[node]) {
				if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo =
					std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp))
				{
					logger->logAttribute(node->name() + ".Pinj", trafo->attributeTyped<Real>("p_inj"));
					logger->logAttribute(node->name() + ".Qinj", trafo->attributeTyped<Real>("q_inj"));
					break;
				}

			}
		}
	}

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(system);
	sim.setTimeStep(time_step);
	sim.setFinalTime(time_end);
	sim.setDomain(Domain::SP);
	sim.setSolverType(Solver::Type::NRP);
	sim.setSolverAndComponentBehaviour(Solver::Behaviour::Simulation);
	sim.doInitFromNodesAndTerminals(true);
	sim.addLogger(logger);

	sim.run();

	return 0;
}

