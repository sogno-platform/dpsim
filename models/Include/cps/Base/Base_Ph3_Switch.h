/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>
#include <cps/AttributeList.h>
namespace CPS {
namespace Base {
namespace Ph3 {
	/// Dynamic Phasor Three-Phase Switch
	class Switch {
	public:
		/// Resistance if switch is open [ohm]
		CPS::Attribute<Matrix>::Ptr mOpenResistance;
		/// Resistance if switch is closed [ohm]
		CPS::Attribute<Matrix>::Ptr mClosedResistance;
		/// Defines if Switch is open or closed
		CPS::Attribute<Bool>::Ptr mSwitchClosed;
		///
		void setParameters(Matrix openResistance, Matrix closedResistance, Bool closed = false) {
			**mOpenResistance = openResistance;
			**mClosedResistance = closedResistance;
			**mSwitchClosed = closed;
		}
		void closeSwitch() { **mSwitchClosed = true; }
		void openSwitch() { **mSwitchClosed = false; }
	};
}
}
}
