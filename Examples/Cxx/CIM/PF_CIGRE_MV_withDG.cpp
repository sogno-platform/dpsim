#include "cps/CIM/Reader.h"
#include <DPsim.h>
#include <cps/CSVReader.h>
#include "../Examples.h"

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

int main(int argc, char** argv){

	// Find CIM files
	std::list<fs::path> filenames;
	std::cout<<std::experimental::filesystem::current_path()<<std::endl;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"Rootnet_FULL_NE_28J17h_DI.xml",
			"Rootnet_FULL_NE_28J17h_EQ.xml",
			"Rootnet_FULL_NE_28J17h_SV.xml",
			"Rootnet_FULL_NE_28J17h_TP.xml"
		}, "dpsim/Examples/CIM/grid-data/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_noLoad1_LeftFeeder_With_LoadFlow_Results", "CIMPATH");
	}
	else {
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	// Simulation parameters
	Real timeStep = 1;
	Real finalTime = 2;
	String simName = "PF_CIGRE_MV_withDG";
	Examples::Grids::CIGREMV::ScenarioConfig scenario;
	Logger::setLogDir("logs/" + simName);

	// read original network topology
    CIM::Reader reader(simName, Logger::Level::debug, Logger::Level::debug);
    SystemTopology system = reader.loadCIM(scenario.systemFrequency, filenames, Domain::SP);
	Examples::Grids::CIGREMV::addInvertersToCIGREMV(system, scenario, Domain::SP);

    auto loggerPF = DPsim::DataLogger::make(simName);
    for (auto node : system.mNodes)
    {
        loggerPF->addAttribute(node->name() + ".V", node->attribute("v"));
    }
    Simulation simPF(simName, Logger::Level::debug);
	simPF.setSystem(system);
	simPF.setTimeStep(timeStep);
	simPF.setFinalTime(finalTime);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(true);
    simPF.addLogger(loggerPF);
    simPF.run();
}