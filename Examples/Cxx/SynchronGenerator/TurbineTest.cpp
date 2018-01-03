/** Synchron Generator Tests
*
* @file
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

#include "Simulation.h"
#include "Components/TurbineGovernor.h"

using namespace DPsim;


// ##################################### Test only Turbine ##################################
int main() {
	// Define Object for saving data on a file
	Logger TurbineOut("TurbineOutput_DPsim.csv");

	// Define machine parameters in per unit
	Real nomPower = 555e6;
	Real nomPhPhVoltRMS = 24e3;
	Real nomFreq = 60;
	Real nomFieldCurr = 1300;
	Int poleNum = 2;
	Real J = 2.8898e+04;
	Real H = 3.7;

	// Turbine
	Real Ta_t = 0.3;
	Real Fa = 0.3;
	Real Tb = 7;
	Real Fb = 0.3;
	Real Tc = 0.2;
	Real Fc = 0.4;
	Real Tsr = 0.1;
	Real Tsm = 0.3;
	Real Kg = 20;

	Real initActivePower = 555e3;

	TurbineGovernor mTurbineGovernor;

	mTurbineGovernor = TurbineGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm);

	Real OmRef = 1;
	Real PmRef = 0.001;

	std::string line;
	Real Om;
	std::ifstream omega ("omega_input.csv");
	Real dt = 0.00001;
	Real t = 0;
	Real Pm = 0;

	mTurbineGovernor.init(PmRef, initActivePower / nomPower);

	while (getline(omega, line))
	{	
		t = t + dt;
		Om = std::stod(line);
		std::cout << Om << '\n';
		Pm = mTurbineGovernor.step(Om, OmRef, PmRef, dt);
		TurbineOut.LogDataLine(t,Pm);
	}


	



	//// Set up simulation
	//Real tf, dt, t;
	//Real om = 2.0*M_PI*60.0;
	//tf = 10; dt = 0.0001; t = 0;
	//Int downSampling = 1;

	//BaseComponent::List circElements;
	//Simulation newSim(circElements, om, dt, tf, log, SimulationType::DynPhasor, downSampling);


	//std::cout << "A matrix:" << std::endl;
	//std::cout << newSim.getSystemMatrix() << std::endl;
	//std::cout << "vt vector:" << std::endl;
	//std::cout << newSim.getLeftSideVector() << std::endl;
	//std::cout << "j vector:" << std::endl;
	//std::cout << newSim.getRightSideVector() << std::endl;

	//Real lastLogTime = 0;
	//Real logTimeStep = 0.0001;

	// Main Simulation Loop
	//while (newSim.getTime() < tf) {
	//	std::cout << newSim.getTime() << std::endl;

	//	mTurbineGovernor.step(Om, OmRef, PmRef, dt);
	//	newSim.increaseByTimeStep();
	//}

	//std::cout << "Simulation finished." << std::endl;

	return 0;
}