// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim-villas/InterfaceShmem.h>

using namespace DPsim;
using namespace CPS::DP::Ph1;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {

	CommandLineArgs args(argc, argv);

	// Find CIM files
	std::list<fs::path> filenames;
	if (argc <= 1) {
		filenames = DPsim::Utils::findFiles({
			"WSCC-09_RX_DI.xml",
			"WSCC-09_RX_EQ.xml",
			"WSCC-09_RX_SV.xml",
			"WSCC-09_RX_TP.xml"
		}, "Examples/CIM/WSCC-09_RX", "CIMPATH");
	}
	else {
		filenames = args.positionalPaths();
	}

	String simName = "Shmem_WSCC-9bus_Ctrl";
	CPS::Logger::setLogDir("logs/"+simName);

	CPS::CIM::Reader reader(CPS::Logger::Level::info, CPS::Logger::Level::off, CPS::Logger::Level::off);
	SystemTopology sys = reader.loadCIM(60, filenames);

	// Extend system with controllable load (Profile)
	auto load_profile = PQLoadCS::make("load_cs_profile");
	load_profile->connect({ sys.node<CPS::DP::SimNode>("BUS6") });
	load_profile->setParameters(0, 0, 230000);
	sys.mComponents.push_back(load_profile);

	// Extend system with controllable load
	auto load = PQLoadCS::make("load_cs");
	load->connect({ sys.node<CPS::DP::SimNode>("BUS5") });
	load->setParameters(0, 0, 230000);
	sys.mComponents.push_back(load);

	// Controllers and filter
	std::vector<Real> coefficients_profile = std::vector<Real>(2000, 1./2000);
	std::vector<Real> coefficients = std::vector<Real>(100, 1./100);

	auto filtP_profile = FIRFilter::make("filter_p_profile", coefficients_profile, 0, CPS::Logger::Level::off);
	load_profile->mActivePower->setReference(filtP_profile->mOutput);

	sys.mComponents.push_back(filtP_profile);

	auto filtP = FIRFilter::make("filter_p", coefficients, 0, CPS::Logger::Level::off);
	load->mActivePower->setReference(filtP->mOutput);
	sys.mComponents.push_back(filtP);

	RealTimeSimulation sim(simName, CPS::Logger::Level::off);
	sim.setSystem(sys);
	sim.setTimeStep(args.timeStep);
	sim.setFinalTime(args.duration);
	sim.setDomain(Domain::DP);
	sim.setSolverType(Solver::Type::MNA);
	sim.doInitFromNodesAndTerminals(true);

	InterfaceShmem intf("/dpsim1-villas", "/villas-dpsim1", nullptr, false);

	auto logger = CPS::DataLogger::make(simName);

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : sys.mNodes) {
		UInt i;
		if (sscanf(n->name().c_str(), "BUS%u", &i) != 1) {
			std::cerr << "Failed to determine bus no of bus: " << n->name() << std::endl;
			continue;
		}

		i--;

		auto n_dp = std::dynamic_pointer_cast<CPS::DP::SimNode>(n);
		auto v = n_dp->mVoltage->deriveCoeff<Complex>(0, 0);

		std::cout << "Signal " << (i*2)+0 << ": Mag  " << n->name() << std::endl;
		std::cout << "Signal " << (i*2)+1 << ": Phas " << n->name() << std::endl;

		intf.exportReal(v->deriveMag(),   (i*2)+0); o++;
		intf.exportReal(v->derivePhase(), (i*2)+1); o++;

		logger->logAttribute(fmt::format("mag_{}", i), v->deriveMag());
		logger->logAttribute(fmt::format("phase_{}", i), v->derivePhase());
	}

	logger->logAttribute("v3", sys.node<CPS::DP::SimNode>("BUS3")->mVoltage);

	// TODO gain by 20e8
	filtP->setInput(intf.importReal(0));
	filtP_profile->setInput(intf.importReal(1));

	intf.exportReal(load->attributeTyped<Real>("P"), o++);
	intf.exportReal(load_profile->attributeTyped<Real>("P"), o++);

	sim.addInterface(std::shared_ptr<Interface>(&intf));
	sim.addLogger(logger);
	sim.run(args.startTime);

	return 0;
}
