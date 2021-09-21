/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
