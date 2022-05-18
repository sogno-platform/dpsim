/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/CIM/Reader.h>
#include <DPsim.h>
#include <dpsim-villas/InterfaceShmem.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv) {
	CommandLineArgs args(argc, argv);

	// Find CIM files
	std::list<fs::path> filenames;
	filenames = DPsim::Utils::findFiles({
		"Rootnet_FULL_NE_06J16h_DI.xml",
		"Rootnet_FULL_NE_06J16h_EQ.xml",
		"Rootnet_FULL_NE_06J16h_SV.xml",
		"Rootnet_FULL_NE_06J16h_TP.xml"
	}, "build/_deps/cim-data-src/CIGRE_MV/NEPLAN/CIGRE_MV_no_tapchanger_With_LoadFlow_Results", "CIMPATH");
	
	String simName = "Shmem_CIGRE_MV_PowerFlowTest";
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, CPS::Logger::Level::debug, CPS::Logger::Level::off);
    SystemTopology sys = reader.loadCIM(system_freq, filenames, CPS::Domain::SP);

	RealTimeSimulation sim(simName, args.logLevel);
	sim.setSystem(sys);
	sim.setTimeStep(args.timeStep);
	sim.setFinalTime(args.duration);
	sim.setDomain(args.solver.domain);
	sim.setSolverType(args.solver.type);
	InterfaceShmem intf("/dpsim1-villas", "/villas-dpsim1");

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : sys.mNodes) {
		UInt i;
		if (sscanf(n->name().c_str(), "N%u", &i) != 1) {
			std::cerr << "Failed to determine bus no of bus: " << n->name() << std::endl;
			continue;
		}

		auto n_stat = std::dynamic_pointer_cast<CPS::SP::SimNode>(n);
		auto v = n_stat->mVoltage->deriveCoeff<Complex>(0, 0);

		std::cout << "Signal " << (i*2)+0 << ": Mag  " << n->name() << std::endl;
		std::cout << "Signal " << (i*2)+1 << ": Phas " << n->name() << std::endl;

		intf.exportReal(v->deriveMag(),   (i*2)+0); o++;
		intf.exportReal(v->derivePhase(), (i*2)+1); o++;
	}

	sim.addInterface(&intf, false);

	sim.run(std::chrono::seconds(5));

	return 0;
}

