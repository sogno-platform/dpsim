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
	class VoltageSource {
	public:
		/// Voltage set point [V]
		const Attribute<Complex>::Ptr mVoltageRef;
		/// Source frequency [Hz]
		const Attribute<Real>::Ptr mSrcFreq;

		explicit VoltageSource(CPS::AttributeBase::Map &attributeList) :
			mVoltageRef(Attribute<Complex>::create("V_ref", attributeList)),
			mSrcFreq(Attribute<Real>::create("f_src", attributeList, -1)) { };

		/// Sets model specific parameters
		void setParameters(Complex voltageRef, Real srcFreq = -1) {
			**mVoltageRef = voltageRef;
			**mSrcFreq = srcFreq;
		}
	};
}
}
}
