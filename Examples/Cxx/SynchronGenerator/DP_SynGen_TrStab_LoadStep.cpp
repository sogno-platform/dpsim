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

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Define simulation parameters
	Real timeStep = 0.0005;
	Real finalTime = 0.1;
	String name = "DP_SynGen_TrStab_Step";
	// Define machine parameters in per unit
	Real nomPower = 555e6;
	Real nomPhPhVoltRMS = 24e3;
	Real nomFreq = 60;
	Real H = 3.7;
	Real Ll = 0.15;
	Real Lmd = 1.6599;
	Real Llfd = 0.1648;
	// Initialization parameters
	Complex initElecPower = Complex(300e6, 0);
	Real initTerminalVolt = 24000;
	Real initVoltAngle = 0;
	Complex initVoltage = Complex(initTerminalVolt * cos(initVoltAngle), initTerminalVolt * sin(initVoltAngle));
	Real mechPower = 300e6;
	// Define grid parameters
	Real Rload = 1.92;
	Real RloadStep = 0.7;

	// Nodes
	auto n1 = Node::make("n1", PhaseType::Single, std::vector<Complex>{ initVoltage });

	// Components
	auto gen = Ph1::SynchronGeneratorTrStab::make("DP_SynGen_TrStab_Step_SynGen");
	gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);
	gen->connect({n1});
	gen->setInitialValues(initElecPower, mechPower);

	auto load = Ph1::Switch::make("DP_SynGen_TrStab_Step_StepLoad");
	load->setParameters(Rload, RloadStep);
	load->connect({Node::GND, n1});
	load->open();

	// System
	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, load});

	// Simulation
	auto logger = DataLogger::make(name);//added
	logger->addAttribute("Current", gen->attribute("i_intf")); //Added
	
	Simulation sim(name, sys, timeStep, finalTime,
		Domain::DP, Solver::Type::MNA, Logger::Level::INFO);

	sim.addLogger(logger);//added
	// Events
	auto sw1 = SwitchEvent::make(0.05, load, true);

	sim.addEvent(sw1);

	sim.run();

	return 0;
}
