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
namespace Ph3 {
	/// Dynamic Phasor Three-Phase Switch
	class Switch {
	public:
		/// Resistance if switch is open [ohm]
		const CPS::Attribute<Matrix>::Ptr mOpenResistance;
		/// Resistance if switch is closed [ohm]
		const CPS::Attribute<Matrix>::Ptr mClosedResistance;
		/// Defines if Switch is open or closed
		const CPS::Attribute<Bool>::Ptr mSwitchClosed;

		explicit Switch(CPS::AttributeList::Ptr attributeList) :
			mOpenResistance(attributeList->create<Matrix>("R_open")),
			mClosedResistance(attributeList->create<Matrix>("R_closed")),
			mSwitchClosed(attributeList->create<Bool>("is_closed")) { };

		///
		void setParameters(Matrix openResistance, Matrix closedResistance, Bool closed = false) {
			**mOpenResistance = openResistance;
			**mClosedResistance = closedResistance;
			**mSwitchClosed = closed;
		}
		void closeSwitch() { **mSwitchClosed = true; }
		void openSwitch() { **mSwitchClosed = false; }
		
		/// Check if switch is closed
		Bool isClosed() { return mSwitchClosed; }
	};
}
}
}
