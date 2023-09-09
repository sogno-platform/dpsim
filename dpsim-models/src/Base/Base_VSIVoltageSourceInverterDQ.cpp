/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_VSIVoltageSourceInverterDQ.h>

using namespace CPS;

void Base::VSIVoltageSourceInverterDQ::setParameters(Real sysOmega, Real VdRef, Real VqRef) {
	**mOmegaN = sysOmega;
	**mVdRef = VdRef;
	**mVqRef = VqRef;
}

void Base::VSIVoltageSourceInverterDQ::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	mTransformerNominalVoltageEnd1 = nomVoltageEnd1;
	mTransformerNominalVoltageEnd2 = nomVoltageEnd2;
	mTransformerRatedPower = ratedPower;
	mTransformerResistance = resistance;
	mTransformerInductance = inductance;
	mTransformerRatioAbs = ratioAbs;
	mTransformerRatioPhase = ratioPhase;
};

void Base::VSIVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc) {
	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;
}

void Base::VSIVoltageSourceInverterDQ::setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega) {
	mKiVoltageCtrl = Ki_voltageCtrl;
	mKiCurrCtrl = Ki_currCtrl;
	mKpVoltageCtrl = Kp_voltageCtrl;
	mKpCurrCtrl = Kp_currCtrl;
	mOmegaVSI = Omega;
}
