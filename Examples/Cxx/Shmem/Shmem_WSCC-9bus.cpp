/** CIM Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <iostream>
#include <list>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {

	// Specify CIM files
#ifdef _WIN32
	String path("Examples\\CIM\\WSCC-09_RX\\");
#elif defined(__linux__) || defined(__APPLE__)
	String path("Examples/CIM/WSCC-09_RX/");
#endif

	std::list<String> filenames = {
		path + "WSCC-09_RX_DI.xml",
		path + "WSCC-09_RX_EQ.xml",
		path + "WSCC-09_RX_SV.xml",
		path + "WSCC-09_RX_TP.xml"
	};

	String simName = "Shmem_WSCC-9bus";

	CIMReader reader(simName, Logger::Level::INFO, Logger::Level::INFO);
	SystemTopology sys = reader.loadCIM(60, filenames);

	RealTimeSimulation sim(simName, sys, 0.001, 120,
		Domain::DP, Solver::Type::MNA, Logger::Level::DEBUG, true);

	// Create shmem interface
	Interface::Config conf;
	conf.samplelen = 64;
	conf.queuelen = 1024;
	conf.polling = false;
	String in  = "/villas-dpsim";
	String out = "/dpsim-villas";
	Interface intf(out, in, &conf);

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : sys.mNodes) {
		auto v = n->attributeComplex("v");

		intf.addExport(v->mag(),   o+0);
		intf.addExport(v->phase(), o+1);

		o += 2;
	}

	sim.addInterface(&intf);
	sim.run();

	return 0;
}
