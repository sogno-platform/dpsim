/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>

namespace CPS {
namespace Base {
namespace Ph1 {
	class VoltageSource {
	protected:
		/// Voltage set point [V]
		Complex mVoltageRef;
		/// Source frequency [Hz]
		Real mSrcFreq = -1;
	public:
		/// Sets model specific parameters
		void setParameters(Complex voltageRef, Real srcFreq = -1) {
			mVoltageRef = voltageRef;
			mSrcFreq = srcFreq;
		}
	};
}
}
}
