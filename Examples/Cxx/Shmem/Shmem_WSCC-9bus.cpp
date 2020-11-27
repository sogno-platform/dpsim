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

#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim/InterfaceShmem.h>

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

	CIMReader reader(simName, Logger::Level::info, Logger::Level::info);
	SystemTopology sys = reader.loadCIM(60, filenames);

	RealTimeSimulation sim(simName, sys, 0.001, 120,
		Domain::DP, Solver::Type::MNA, Logger::Level::debug, true);

	InterfaceShmem intf("/dpsim-villas", "/villas-dpsim");

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : sys.mNodes) {
		auto v = n->attributeComplex("v");

		intf.exportReal(v->mag(),   o+0);
		intf.exportReal(v->phase(), o+1);

		o += 2;
	}

	sim.addInterface(&intf);
	sim.run();

	return 0;
}
