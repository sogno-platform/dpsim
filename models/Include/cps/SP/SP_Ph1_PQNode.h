/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>


namespace CPS {
namespace SP {
namespace Ph1 {

	class PQNode: public SimPowerComp<Complex>, public SharedFactory<PQNode>{
	private:
		Real mPowerNom;
		Real mReactivePowerNom;
		Real mPower;
		Real mReactivePower;


		Real mVoltageAbsPerUnit;
		Complex mVoltagePerUnit;

	public:
		PQNode(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);

        PQNode(String uid, String name, Real power, Real reactive_power,
            Logger::Level logLevel = Logger::Level::off);


		void initializeFromTerminal();
		void setPerUnitSystem();

	};




}
}
}
