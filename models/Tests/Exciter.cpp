/** Synchron Generator Excitor Test
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
* @license GNU General Public License (version 3)
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

#include <fstream>

#include <cps/Signal/Exciter.h>

using namespace CPS;

int main(int argc, char* argv[]) {
	// Define Object for saving data on a file
	std::ofstream log("ExciterOutput.csv");
	std::ifstream vd("vd_pu.csv");
	std::ifstream vq("vq_pu.csv");

	// Generator parameters
	Real Lmd = 1.6599;
	Real Rfd = 0.00059987;

	// Initialize generator
	Real Vf_init = 1;
	Real Vh_init = 1;

	// Exciter
	Real Ka = 46;
	Real Ta = 0.06;
	Real Ke = -0.043478260869565223;
	Real Te = 0.46;
	Real Kf = 0.1;
	Real Tf = 1;
	Real Tr = 0.02;
	Real Vref = 1;

	Signal::Exciter exciter("exciter");
	exciter.setParameters(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lmd, Rfd);

	// Variables to read input
	std::string line_vd;
	std::string line_vq;
	Real mVd;
	Real mVq;

	// Time step and time
	Real dt = 0.00005;
	Real t = 0;

	// Exciter output
	Real vt = 0;

	while (getline(vd, line_vd) && getline(vq, line_vq)) {
		t = t + dt;
		mVd = std::stod(line_vd);
		mVq = std::stod(line_vq);
		std::cout << t << '\n';

		if (t == dt) {
			exciter.initialize(Vh_init, Vf_init);
		}

		vt = exciter.step(mVd, mVq, Vref, dt);

		log << t << "," << vt * 257198.07031934269 << std::endl;
	}

	return 0;
}
