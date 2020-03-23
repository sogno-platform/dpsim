/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_Inverter.h>

using namespace CPS;
using namespace CPS::DP;

int main(int argc, char *argv[]) {
	String simName = "Inverter_test";
	Logger::setLogDir("logs/"+simName);

	auto inv = Ph1::Inverter::make("inv", Logger::Level::debug);
	inv->setParameters( std::vector<CPS::Int>{2,2,2,2,4,4,4,4},
						std::vector<CPS::Int>{-3,-1,1,3,-5,-1,1,5},
						360, 0.87, 0);
	//inv->initialize(50, );
	inv->calculatePhasors();
}
