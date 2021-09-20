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
namespace Ph3 {
	class VoltageSource {
	protected:
	public:
		/// Sets model specific parameters
		virtual void setParameters(Complex voltageRef, Real srcFreq = -1) {}=0;
        virtual void updateVoltage()=0;
	};
}
}
}
