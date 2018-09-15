/** SynGenVBR Example
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
using namespace DPsim::EMT::Ph3;

int main(int argc, char* argv[]) {
	// Define machine parameters in per unit
	Real nomPower = 555e6;
	Real nomPhPhVoltRMS = 24e3;
	Real nomFreq = 60;
	Real nomFieldCurr = 1300;
	Int poleNum = 2;
	Real J = 2.8898e+04;
	Real H = 3.7;

	Real Rs = 0.003;
	Real Ll = 0.15;
	Real Lmd = 1.6599;
	Real Lmd0 = 1.6599;
	Real Lmq = 1.61;
	Real Lmq0 = 1.61;
	Real Rfd = 0.0006;
	Real Llfd = 0.1648;
	Real Rkd = 0.0284;
	Real Llkd = 0.1713;
	Real Rkq1 = 0.0062;
	Real Llkq1 = 0.7252;
	Real Rkq2 = 0.0237;
	Real Llkq2 = 0.125;
	//Real Rkq2 = 0;
	//Real Llkq2 = 0;

	for (Real i = 0.00005; i <= 0.01; i = i + 0.00005) {
		if (i > 0.0001)
			i = i + 0.00005;
		if (i >= 0.0011)
			i = i + 0.0009;

		String mGeneratorName = "EMT_VBR_" + std::to_string(i);
		// Declare circuit components
		Component::Ptr gen = VoltageBehindReactanceEMTNew::make(mGeneratorName, 0, 1, 2,
			nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
			Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, Logger::Level::INFO);

			Real loadRes = 1.92;
			Component::Ptr r1 = Resistor::make("r1", 0, GND, loadRes);
			Component::Ptr r2 = Resistor::make("r2", 1, GND, loadRes);
			Component::Ptr r3 = Resistor::make("r3", 2, GND, loadRes);

			SystemComponentList comps = { gen, r1, r2, r3 };

			// Declare circuit components for resistance change
			Real breakerRes = 19.2 + 0.001;
			Component::Ptr rBreaker1 = Resistor::make("rbreak1", 0, GND, breakerRes);
			Component::Ptr rBreaker2 = Resistor::make("rbreak2", 1, GND, breakerRes);
			Component::Ptr rBreaker3 = Resistor::make("rbreak3", 2, GND, breakerRes);

			SystemComponentList compsBreakerOn = { gen, rBreaker1, rBreaker2, rBreaker3, r1, r2, r3 };
			// Set up simulation
			Real tf, dt, t;
			Real om = 2.0*M_PI*60.0;
			dt = i; t = 0;
			tf = 0.3;
			Int downSampling = 1;
			String mSimulationName = "EMT_SynchronGenerator_VBR_" + std::to_string(i);
			SynGenSimulation sim(mSimulationName, comps, om, dt, tf, Logger::Level::INFO, SimulationType::EMT, downSampling);
			sim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);
			sim.addSystemTopology(compsBreakerOn);
			sim.switchSystemMatrix(0);

			// Initialize generator
			Real initActivePower = 300e6;
			Real initReactivePower = 0;
			Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
			Real initVoltAngle = -DPS_PI / 2;
			Real fieldVoltage = 7.0821;
			Real mechPower = 300e6;
			auto genPtr = std::dynamic_pointer_cast<EMT::Ph3::VoltageBehindReactanceEMTNew>(gen);
			genPtr->initialize(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

			std::cout << "A matrix:" << std::endl;
			std::cout << sim.systemMatrix() << std::endl;
			std::cout << "vt vector:" << std::endl;
			std::cout << sim.leftSideVector() << std::endl;
			std::cout << "j vector:" << std::endl;
			std::cout << sim.rightSideVector() << std::endl;

			Real lastLogTime = 0;
			Real logTimeStep = 0.00005;
			sim.setSwitchTime(0.1, 1);
			sim.setSwitchTime(0.2, 0);

			sim.run();
	}

	return 0;
}
