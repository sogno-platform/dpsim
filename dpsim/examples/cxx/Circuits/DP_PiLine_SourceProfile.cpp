/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include <dpsim-models/CSVReader.h>

using namespace DPsim;
using namespace CPS::DP;

void simPiLineSource() {

#ifdef _WIN32
	String loadProfilePath("build\\_deps\\profile-data-src\\NetworkInjektion_Voltage\\");
#elif defined(__linux__) || defined(__APPLE__)
	String loadProfilePath("build/_deps/profile-data-src/NetworkInjektion_Voltage/");
#endif

	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_PiLine_Source";
	Logger::setLogDir("logs/"+simName);

	// read csv
	String ni_name = "vs_test";
	CPS::CSVReader csvreader(ni_name + ".csv", loadProfilePath, Logger::Level::debug);

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");

	// Components
	auto vs = Ph1::NetworkInjection::make(ni_name);
	vs->setParameters(CPS::Math::polar(100000, 0));
	
	// Parametrization of components
	Real resistance = 5;
	Real inductance = 0.16;
	Real capacitance = 1.0e-6;
	Real conductance = 1e-6;

	auto line = Ph1::PiLine::make("Line");
	line->setParameters(resistance, inductance, capacitance, conductance);

	auto load = Ph1::Resistor::make("R_load");
	load->setParameters(10000);

	// Topology
	vs->connect({ n1 });
	line->connect({ n1, n2 });
	load->connect({ n2, SimNode::GND });

	auto sys = SystemTopology(50,
		SystemNodeList{n1, n2},
		SystemComponentList{vs, line, load});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("iline", line->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	std::vector<std::shared_ptr<CPS::DP::Ph1::NetworkInjection>> networkinjections;
	networkinjections.push_back(vs);
	csvreader.assignSourceProfile(networkinjections, sim.time(), sim.timeStep(), sim.finalTime(), CPS::CSVReader::Mode::AUTO);

	sim.run();
}


int main(int argc, char* argv[]) {
	simPiLineSource();
}
