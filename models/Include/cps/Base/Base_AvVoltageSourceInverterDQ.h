/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>
#include <cps/TopologicalPowerComp.h>

namespace CPS {
namespace Base {
/// @brief Base model of average inverter
	class AvVoltageSourceInverterDQ :
        public virtual TopologicalPowerComp {
	protected:
        /// Complex nominal voltage [V]
		Real mVnom;

		/// Power parameters
		Real mPref;
		Real mQref;

		/// filter parameters
		Real mLf;
		Real mCf;
		Real mRf;
		Real mRc;

		/// transformer
		Real mTransformerResistance;
		Real mTransformerInductance;
		Real mTransformerRatioAbs;
		Real mTransformerRatioPhase;

		/// PLL
		Real mOmegaN;
		Real mKiPLL;
		Real mKpPLL;

		/// Power controller
		Real mOmegaCutoff;
		Real mKiPowerCtrld;
		Real mKiPowerCtrlq;
		Real mKpPowerCtrld;
		Real mKpPowerCtrlq;

		/// Current controller
		Real mKiCurrCtrld;
		Real mKiCurrCtrlq;
		Real mKpCurrCtrld;
		Real mKpCurrCtrlq;

		/// states
		Real mThetaPLL = 0;
		Real mPhiPLL;
		Real mP;
		Real mQ;
		Real mPhi_d;
		Real mPhi_q;
		Real mGamma_d;
		Real mGamma_q;

		/// state space matrices
		Matrix mA = Matrix::Zero(8, 8);
		Matrix mB = Matrix::Zero(8, 7);
		Matrix mC = Matrix::Zero(2, 8);
		Matrix mD = Matrix::Zero(2, 7);

    public:
		///
		AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off);

        /// Setter for general parameters
		void setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref);
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff);
		/// Setter for filter parameters
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		/// Setter for optional connection transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
			Real ratioPhase, Real resistance, Real inductance, Real omega);
    };
}
}
