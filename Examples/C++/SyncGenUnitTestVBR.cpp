/** Synchron Generator Unit Test (VBR)
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

#include "SyncGenUnitTestVBR.h"

#include "Simulation.h"
#include "Utilities.h"

using namespace DPsim;

void DPsim::SynGenUnitTestVBR() {

	// Define Object for saving data on a file
	Logger log("Logs/logVBR.txt"),
		synGenLogVolt("Logs/data_synGenVBR_v.csv"),
		synGenLogCurr("Logs/data_synGenVBR_i.csv"),
		synGenLogElecTorque("Logs/data_synGenVBR_Te.csv"),
		synGenLogOmega("Logs/data_synGenVBR_omega.csv"),
		synGenLogTheta("Logs/data_synGenVBR_theta.csv");

	// Define machine parameters in per unit
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 2.8898e+04;
	double H = 3.7;

	double Rs = 0.003;
	double Ll = 0.15;
	double Lmd = 1.6599;
	double Lmd0 = 1.6599;
	double Lmq = 1.61;
	double Lmq0 = 1.61;
	double Rfd = 0.0006;
	double Llfd = 0.1648;
	double Rkd = 0.0284;
	double Llkd = 0.1713;
	double Rkq1 = 0.0062;
	double Llkq1 = 0.7252;
	//double Rkq2 = 0.0237;
	//double Llkq2 = 0.125;
	double Rkq2 = 0;
	double Llkq2 = 0;


	// Declare circuit components
	BaseComponent* gen = new VoltageBehindReactanceEMT("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.3; dt = 0.000001; t = 0;
	Int downSampling = 25;
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::EMT);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((VoltageBehindReactanceEMT*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	//std::cout << "A matrix:" << std::endl;
	//std::cout << newSim.getSystemMatrix() << std::endl;
	//std::cout << "vt vector:" << std::endl;
	//std::cout << newSim.getLeftSideVector() << std::endl;
	//std::cout << "j vector:" << std::endl;
	//std::cout << newSim.getRightSideVector() << std::endl;

	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorVBR(log, gen, synGenLogVolt, synGenLogCurr, synGenLogElecTorque, synGenLogOmega, synGenLogTheta, fieldVoltage, mechPower, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
}

void DPsim::SynGenUnitTestVBRDP() {

	// Define Object for saving data on a file
	Logger log("Logs/logVBR.txt"),
		synGenLogVolt("Logs/data_synGenVBR_DP_v.csv"),
		synGenLogCurr("Logs/data_synGenVBR_DP_i.csv"),
		synGenLogElecTorque("Logs/data_synGenVBR_DP_Te.csv"),
		synGenLogOmega("Logs/data_synGenVBR_DP_omega.csv"),
		synGenLogTheta("Logs/data_synGenVBR_DP_theta.csv");

	// Define machine parameters in per unit
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 2.8898e+04;
	double H = 3.7;

	double Rs = 0.003;
	double Ll = 0.15;
	double Lmd = 1.6599;
	double Lmd0 = 1.6599;
	double Lmq = 1.61;
	double Lmq0 = 1.61;
	double Rfd = 0.0006;
	double Llfd = 0.1648;
	double Rkd = 0.0284;
	double Llkd = 0.1713;
	double Rkq1 = 0.0062;
	double Llkq1 = 0.7252;
	//double Rkq2 = 0.0237;
	//double Llkq2 = 0.125;
	double Rkq2 = 0;
	double Llkq2 = 0;



	// Declare circuit components
	BaseComponent* gen = new VoltageBehindReactanceDP("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.3; dt = 0.000005; t = 0;
	Int downSampling = 25;
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::DynPhasor);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((VoltageBehindReactanceDP*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	//std::cout << "A matrix:" << std::endl;
	//std::cout << newSim.getSystemMatrix() << std::endl;
	//std::cout << "vt vector:" << std::endl;
	//std::cout << newSim.getLeftSideVector() << std::endl;
	//std::cout << "j vector:" << std::endl;
	//std::cout << newSim.getRightSideVector() << std::endl;

	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorVBR(log, gen, synGenLogVolt, synGenLogCurr, synGenLogElecTorque, synGenLogOmega, synGenLogTheta, fieldVoltage, mechPower, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
}
