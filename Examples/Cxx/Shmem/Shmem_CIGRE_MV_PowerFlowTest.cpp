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

#include "cps/CIM/Reader.h"
#include <DPsim.h>

using namespace std;
using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;


/*
 * This example runs the powerflow for the CIGRE MV benchmark system (neglecting the tap changers of the transformers)
 */
int main(int argc, char** argv){
	CommandLineArgs args(argc, argv);

	#ifdef _WIN32
		String path("..\\..\\..\\..\\dpsim\\Examples\\CIM\\CIGRE_MV_NoTap\\");
	#elif defined(__linux__) || defined(__APPLE__)
		String path("Examples/CIM/CIGRE_MV_NoTap/");
	#endif

	std::list<string> filenames = {
	path + "Rootnet_FULL_NE_06J16h_DI.xml",
	path + "Rootnet_FULL_NE_06J16h_EQ.xml",
	path + "Rootnet_FULL_NE_06J16h_SV.xml",
	path + "Rootnet_FULL_NE_06J16h_TP.xml"
	};
	String simName = "Shmem_CIGRE-MV-NoTap";
	CPS::Real system_freq = 50;

    CIM::Reader reader(simName, Logger::Level::DEBUG, Logger::Level::NONE);
    SystemTopology sys = reader.loadCIM(system_freq, filenames, CPS::Domain::Static);

	RealTimeSimulation sim(simName, sys, args.timeStep, args.duration, args.solver.domain, args.solver.type, args.logLevel);

	// Create shmem interface
	Interface::Config conf;
	conf.samplelen = 64;
	conf.queuelen = 1024;
	conf.polling = false;
	String in  = "/villas-dpsim1";
	String out = "/dpsim1-villas";
	Interface intf(out, in, &conf);

	// Register exportable node voltages
	UInt o = 0;
	for (auto n : sys.mNodes) {
		UInt i;
		if (sscanf(n->name().c_str(), "N%u", &i) != 1) {
			std::cerr << "Failed to determine bus no of bus: " << n->name() << std::endl;
			continue;
		}

		auto n_stat = std::dynamic_pointer_cast<CPS::Static::Node>(n);
		auto v = n_stat->attributeMatrix<Complex>("v");
		auto v0 = v->coeffComplex(0,0);

		std::cout << "Signal " << (i*2)+0 << ": Mag  " << n->name() << std::endl;
		std::cout << "Signal " << (i*2)+1 << ": Phas " << n->name() << std::endl;

		intf.addExport(v0->mag(),   (i*2)+0); o++;
		intf.addExport(v0->phase(), (i*2)+1); o++;
	}

	sim.addInterface(&intf, false, false);

	sim.run(std::chrono::seconds(5));

	return 0;
}

