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
#include "Components/Exciter.h"

using namespace DPsim;


// ##################################### Test only Turbine ##################################
int main() {
		// Define Object for saving data on a file
		Logger ExciterOut("ExciterOutput_DPsim.csv");

		//Generator parameters
		Real Lmd = 1.6599;
		Real Rfd = 0.0006;

		// initialize generator
		Real initActivePower = 555e3;
		Real fieldVoltage = 0.00036146923427354219;

		Real nomPower = 555e6;

		//Exciter
		Real Ka = 20;
		Real Ta = 0.2;
		Real Ke = 1;
		Real Te = 0.314;
		Real Kf = 0.063;
		Real Tf = 0.35;
		Real Tr = 0.02;
		Exciter mExciter;
		mExciter = Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lmd, Rfd);
		Real Vref = 1;

		// Variables to read input
		std::string line_vd;
		std::string line_vq;
		Real mVd;
		Real mVq;
		std::ifstream vd("vd_input.csv");
		std::ifstream vq("vq_input.csv");

		// Time step and time
		Real dt = 0.00001;
		Real t = 0;

		// Exciter output
		Real vt = 0;

		//mTurbineGovernor.init(PmRef, initActivePower / nomPower);

		while (getline(vd, line_vd) && getline(vq, line_vq))
		{
				t = t + dt;
				mVd = std::stod(line_vd);
				mVq = std::stod(line_vq);
				std::cout << t << '\n';
				if (t == dt)
				{
						mExciter.init(mVd, mVq, Vref, fieldVoltage);
				}
				vt = mExciter.step(mVd, mVq, Vref, dt);
				ExciterOut.LogDataLine(t, vt*257198.07031934269);
		}

		return 0;
}