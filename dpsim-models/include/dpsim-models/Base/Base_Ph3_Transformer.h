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
	class Transformer {
	protected:
		/// Nominal voltage of primary side
		Real mNominalVoltageEnd1;
		/// Nominal voltage of secondary side
		Real mNominalVoltageEnd2;
		/// Resistance [Ohm]
		Matrix mResistance;
		/// Inductance [H]
		Matrix mInductance;

	public:
		///Transformer ratio
		const Attribute<Complex>::Ptr mRatio;

		explicit Transformer(CPS::AttributeList::Ptr attributeList) :
			mRatio(attributeList->create<Complex>("ratio")) { };

		///
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase, Matrix resistance, Matrix inductance) {
			mNominalVoltageEnd1 = nomVoltageEnd1;
			mNominalVoltageEnd2 = nomVoltageEnd2;
			**mRatio = std::polar<Real>(ratioAbs, ratioPhase);
			mResistance = resistance;
			mInductance = inductance;
		}
	};
}
}
}
