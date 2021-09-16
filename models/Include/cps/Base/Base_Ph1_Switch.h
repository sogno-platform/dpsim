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
	/// Dynamic Phasor Three-Phase Switch
	class Switch {
	protected:
		/// Resistance if switch is open [ohm]
		Real mOpenResistance;
		/// Resistance if switch is closed [ohm]
		Real mClosedResistance;
		/// Defines if Switch is open or closed
		Bool mIsClosed;
	public:
		///
		void setParameters(Real openResistance, Real closedResistance, Bool closed = false) {
			mOpenResistance = openResistance;
			mClosedResistance = closedResistance;
			mIsClosed = closed;
		}

		/// Close switch
		void close() { mIsClosed = true; }
		/// Open switch
		void open() { mIsClosed = false; }
		/// Check if switch is closed
		Bool isClosed() { return mIsClosed; }
	};
}
}
}
