/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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
#include <fstream>

#include <DPsim.h>
#include <dpsim/ThreadLevelScheduler.h>
#include <dpsim/OpenMPLevelScheduler.h>

using namespace DPsim;
using namespace CPS;

Component::List multiply_diakoptics(SystemTopology& sys, int copies,
	Real resistance, Real inductance, Real capacitance, UInt splits = 0) {

    sys.multiply(copies);
	int counter = 0;
    std::vector<String> nodes = {"BUS5", "BUS8", "BUS6"};

    Component::List tear_components;
    UInt splitEvery;

	if (splits > 0)
        splitEvery = UInt(copies+1 / splits);
    else
        splitEvery = 1;

    for (auto orig_node : nodes) {
		std::vector<String> nodeNames{orig_node};
    	for (int i = 2; i < copies+2; i++) {
			nodeNames.push_back(orig_node + "_" + std::to_string(i));
		}
		nodeNames.push_back(orig_node);

		int nlines = copies == 1 ? 1 : copies+1;
        for (int i = 0; i < nlines; i++) {

            auto line = DP::Ph1::PiLine::make("line" + std::to_string(counter));
            line->setParameters(resistance, inductance, capacitance);

            line->connect({sys.node<DP::Node>(nodeNames[i]), sys.node<DP::Node>(nodeNames[i+1])});

			if (i % splitEvery == 0)
                sys.addTearComponent(line);
            else
                sys.addComponent(line);

            counter += 1;
		}
	}
	return tear_components;
}

void simulateDiakoptics(std::list<fs::path> filenames) {
	String simName = "WSCC_9bus_diakoptics";
	Logger::setLogDir("logs/"+simName);

	CIM::Reader reader(simName, Logger::Level::off, Logger::Level::off);
	SystemTopology sys = reader.loadCIM(60, filenames);

	Component::List tearComps = multiply_diakoptics(sys, 3, 12.5, 0.16, 1e-6);

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(0.0001);
	sim.setFinalTime(0.5);
	sim.setDomain(Domain::DP);
	sim.setTearingComponents(sys.mTearComponents);
	sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(4));

	sim.run();
}

int main(int argc, char *argv[]) {
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
		filenames = std::list<fs::path>(argv + 1, argv + argc);
	}

	simulateDiakoptics(filenames);
}
