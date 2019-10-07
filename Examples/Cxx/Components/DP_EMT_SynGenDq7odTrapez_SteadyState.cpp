/** SynGenDPBalancedResLoad Example
 *
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

#include <DPsim.h>

using namespace DPsim;

// Define machine parameters in per unit
Real nomPower = 555e6;
Real nomPhPhVoltRMS = 24e3;
Real nomFreq = 60;
Real nomFieldCurr = 1300;
Int poleNum = 2;
Real H = 3.7;
Real Rs = 0.003;
Real Ll = 0.15;
Real Lmd = 1.6599;
Real Lmq = 1.61;
Real Rfd = 0.0006;
Real Llfd = 0.1648;
Real Rkd = 0.0284;
Real Llkd = 0.1713;
Real Rkq1 = 0.0062;
Real Llkq1 = 0.7252;
Real Rkq2 = 0.0237;
Real Llkq2 = 0.125;
// Initialization parameters
Real initActivePower = 300e6;
Real initReactivePower = 0;
Real initMechPower = 300e6;
Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
Real initVoltAngle = -PI / 2;
Real fieldVoltage = 7.0821;

// Define grid parameters
// resistance for 300 MW output
Real Rload = 1.92;

// Initial node voltage
auto initVoltN1 = std::vector<Complex>({
	Complex(initTerminalVolt * cos(initVoltAngle),
		initTerminalVolt * sin(initVoltAngle)),
	Complex(initTerminalVolt * cos(initVoltAngle - 2 * PI / 3),
		initTerminalVolt * sin(initVoltAngle - 2 * PI / 3)),
	Complex(initTerminalVolt * cos(initVoltAngle + 2 * PI / 3),
		initTerminalVolt * sin(initVoltAngle + 2 * PI / 3)) });

void DP_SynGenDq7odTrapez_SteadyState(Real timeStep, Real finalTime) {
	String simName = "DP_SynGenDq7odTrapez_SteadyState";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::DP::Node::make("n1", PhaseType::ABC, initVoltN1);

	// Components
	auto gen = CPS::DP::Ph3::SynchronGeneratorDQTrapez::make("SynGen");
	gen->setParametersFundamentalPerUnit(nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
		initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, initMechPower);

	auto res = CPS::DP::Ph3::SeriesResistor::make("R_load", Logger::Level::info);
	res->setParameters(Rload);

	// Connections
	gen->connect({n1});
	res->connect({CPS::DP::Node::GND, n1});

	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("i_load", res->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);

	sim.run();
}

void EMT_SynGenDq7odTrapez_SteadyState(Real timeStep, Real finalTime) {
	String simName = "EMT_SynGenDq7odTrapez_SteadyState";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::EMT::Node::make("n1", PhaseType::ABC, initVoltN1);

	// Components
	auto gen = CPS::EMT::Ph3::SynchronGeneratorDQTrapez::make("SynGen");
	gen->setParametersFundamentalPerUnit(nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
		initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, initMechPower);

	auto res = CPS::EMT::Ph3::SeriesResistor::make("R_load", Logger::Level::info);
	res->setParameters(Rload);

	// Connections
	gen->connect({n1});
	res->connect({CPS::EMT::Node::GND, n1});

	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("i_load", res->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {

	Real finalTime = 0.3;
	Real timeStep = 0.00005;
	DP_SynGenDq7odTrapez_SteadyState(timeStep, finalTime);
	EMT_SynGenDq7odTrapez_SteadyState(timeStep, finalTime);
}
