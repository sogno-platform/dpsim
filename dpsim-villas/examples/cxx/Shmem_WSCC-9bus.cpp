// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim-villas/InterfaceShmem.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {

	std::list<fs::path> filenames = DPsim::Utils::findFiles({
		"WSCC-09_RX_DI.xml",
		"WSCC-09_RX_EQ.xml",
		"WSCC-09_RX_SV.xml",
		"WSCC-09_RX_TP.xml"
	}, "Examples/CIM/WSCC-09_RX", "CIMPATH");

	String simName = "Shmem_WSCC-9bus";

	CIMReader reader(CPS::Logger::Level::info, CPS::Logger::Level::off, CPS::Logger::Level::info);
	SystemTopology sys = reader.loadCIM(60, filenames);

	RealTimeSimulation sim(simName, CPS::Logger::Level::debug);
	sim.setSystem(sys);
	sim.setTimeStep(0.001);
	sim.setFinalTime(120);
	sim.setDomain(Domain::DP);
	sim.setSolverType(Solver::Type::MNA);
	sim.doInitFromNodesAndTerminals(true);

	InterfaceShmem intf("/dpsim-villas", "/villas-dpsim");

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : sys.mNodes) {
		auto v = n->attributeTyped<Complex>("v");

		intf.exportReal(v->deriveMag(),   o+0);
		intf.exportReal(v->derivePhase(), o+1);

		o += 2;
	}

	sim.addInterface(std::shared_ptr<Interface>(&intf));
	sim.run();

	return 0;
}
