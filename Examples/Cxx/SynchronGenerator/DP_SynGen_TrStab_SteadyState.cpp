/*********************************************************************************
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
*
* CPowerSystems
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
	Real finalTime = 0.03;
	String name = "DP_SynGen_TrStab_StState";
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

	// Nodes
	auto n1 = Node::make("n1", PhaseType::Single, std::vector<Complex>{ initVoltage });

	// Components
	auto gen = Ph1::SynchronGeneratorTrStab::make("DP_SynGen_TrStab_StState_SynGen", Logger::Level::DEBUG);  
	gen->setParameters(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H, initElecPower, mechPower);
    gen->setNodes(Node::List{n1});
	
	auto res = Ph1::Resistor::make("DP_SynGen_TrStab_StState_Rl", Logger::Level::DEBUG);
	res->setParameters(Rload);
	res->setNodes(Node::List{Node::GND, n1});

	// System
	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res});

	// Simulation
	Simulation sim(name, sys, timeStep, finalTime, 
		Domain::DP, Solver::Type::MNA, Logger::Level::INFO);

	sim.run();

	return 0;
}
