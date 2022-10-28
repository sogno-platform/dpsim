/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>
#include <dpsim-models/AttributeList.h>

namespace CPS {
namespace Base {
namespace Ph1 {
	/// Dynamic Phasor Three-Phase Switch
	class Switch {
	public:
		/// Resistance if switch is open [ohm]
		Attribute<Real>::Ptr mOpenResistance;
		/// Resistance if switch is closed [ohm]
		Attribute<Real>::Ptr mClosedResistance;
		/// Defines if Switch is open or closed
		Attribute<Bool>::Ptr mSwitchClosed;
		///
		void setParameters(Real openResistance, Real closedResistance, Bool closed = false) {
			**mOpenResistance = openResistance;
			**mClosedResistance = closedResistance;
			**mSwitchClosed = closed;
		}

		/// Close switch
		void closeSwitch() { **mSwitchClosed = true; }
		/// Open switch
		void openSwitch() { **mSwitchClosed = false; }
		/// Check if switch is closed
		Bool isClosed() { return **mSwitchClosed; }
	};
}
}
}
