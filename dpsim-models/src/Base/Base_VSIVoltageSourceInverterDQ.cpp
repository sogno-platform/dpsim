/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_VSIVoltageSourceInverterDQ.h>

using namespace CPS;

void Base::VSIVoltageSourceInverterDQ::setParameters(
	Real sysOmega, Real VdRef, Real VqRef) {

	mOmegaNom = sysOmega;
	mVdRef = VdRef;
	mVqRef = VqRef;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nGeneral Parameters:"
		"\n\tNominal Omega = {} [1/s]"
		"\n\tVdRef = {} [V] "
		"\n\tVqRef = {} [V]",
		mOmegaNom, mVdRef, mVqRef);
}

void Base::VSIVoltageSourceInverterDQ::setTransformerParameters(
	Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	mTransformerNominalVoltageEnd1 = nomVoltageEnd1;
	mTransformerNominalVoltageEnd2 = nomVoltageEnd2;
	mTransformerResistance = resistance;
	mTransformerInductance = inductance;
	mTransformerRatioAbs = ratioAbs;
	mTransformerRatioPhase = ratioPhase;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nTransformer Parameters:"
		"\n\tNominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]"
		"\n\tRated Apparent Power = {} [VA]"
		"\n\tResistance={} [Ohm] Inductance={} [H]"
    	"\n\tTap Ratio={} [ ] Phase Shift={} [deg]", 
		mTransformerRatioAbs, mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2,
		mTransformerResistance, mTransformerInductance, mTransformerRatioPhase);
}

void Base::VSIVoltageSourceInverterDQ::setFilterParameters(
	Real Lf, Real Cf, Real Rf, Real Rc) {

	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nFilter Parameters:"
		"\n\tInductance Lf = {} [H]"
		"\n\tCapacitance Cf = {} [F]"
		"\n\tResistance Rf = {} [H]" 
		"\n\tResistance Rc = {} [F]",
		mLf, mCf, mRf, mRc);
}

void Base::VSIVoltageSourceInverterDQ::addVSIController(std::shared_ptr<Base::VSIControlDQ> VSIController) {
	mVSIController = VSIController;
	mWithControl = true;
}