/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Signal/SignalGenerator.h>

namespace CPS {
namespace Signal {
	class SineWaveGenerator :
		public SignalGenerator {
    private:
		///
		//Attribute<Complex>::Ptr mVoltageRef;
		///
		//Attribute<Real>::Ptr mSrcFreq;
    public:
        SineWaveGenerator(String name, Logger::Level logLevel = Logger::Level::off);

        Complex step(Real time);

		Complex getVoltage();
        
		///
		void setParameters(Complex voltageRef, Real srcFreq = -1);
    };
}
}
