/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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
	class Transformer {
	protected:
		/// Nominal voltage of primary side
		Real mNominalVoltageEnd1;
		/// Nominal voltage of secondary side
		Real mNominalVoltageEnd2;
		/// Complex transformer ratio
		Complex mRatio;
		/// Resistance [Ohm]
		Real mResistance;
		/// Inductance [H]
		Real mInductance;

	public:
		///
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance) {
			mNominalVoltageEnd1 = nomVoltageEnd1;
			mNominalVoltageEnd2 = nomVoltageEnd2;
			mRatio = std::polar<Real>(ratioAbs, ratioPhase);
			mResistance = resistance;
			mInductance = inductance;
		}
	};

	class Transformer3W {
	protected:
		/// Nominal voltage of primary side
		Real mNominalVoltageEnd1;
		/// Nominal voltage of secondary side
		Real mNominalVoltageEnd2;
		/// Nominal voltage of secondary side
		Real mNominalVoltageEnd3;
		/// Transformer ratio primary side
		Complex mRatio1;
		/// Transformer ratio secondary side
		Complex mRatio2;
		/// Transformer ratio tetrary side
		Complex mRatio3;
		/// Resistance [Ohm] primary side
		Real mResistance1;
		/// Resistance [Ohm] secondary side
		Real mResistance2;
		/// Resistance [Ohm] tetrary side
		Real mResistance3;
		/// Inductance [H] primary side
		Real mInductance1;
		/// Inductance [H] secondary side
		Real mInductance2;
		/// Inductance [H] tetrary side
		Real mInductance3;

	public:
		///
		void setParameters(
			Real nomVoltageEnd1, Real nomVoltageEnd2, Real nomVoltageEnd3, 
			Real ratioAbs1, Real ratioAbs2, Real ratioAbs3, 
			Real ratioPhase1, Real ratioPhase2, Real ratioPhase3, 
			Real resistance1, Real resistance2, Real resistance3, 
			Real inductance1, Real inductance2, Real inductance3
		) {

			mNominalVoltageEnd1 = nomVoltageEnd1;
			mNominalVoltageEnd2 = nomVoltageEnd2;
			mNominalVoltageEnd3 = nomVoltageEnd3;
			mRatio1 = std::polar<Real>(ratioAbs1, ratioPhase1);
			mRatio2 = std::polar<Real>(ratioAbs2, ratioPhase2);
			mRatio3 = std::polar<Real>(ratioAbs3, ratioPhase3);
			mResistance1 = resistance1;
			mResistance2 = resistance2;
			mResistance3 = resistance3;
			mInductance1 = inductance1;
			mInductance2 = inductance2;
			mInductance3 = inductance3;
		}
	};
}
}
}
