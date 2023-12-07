/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/AttributeList.h>


namespace CPS {
namespace Base {
/// @brief Base model of average inverter
	class VSIVoltageSourceInverterDQ {
	private:
		/// Component logger
		Logger::Log mLogger;

	protected:
		// ### General Parameters ###
		/// Nominal frequency
		const Attribute<Real>::Ptr mOmegaN;
		/// Nominal system angle
		const Attribute<Real>::Ptr mThetaN;
		/// Simulation step
		Real mTimeStep;

		// ### Inverter Parameters ###
		/// Nominal voltage
		Real mVnom;
		/// Voltage d reference
		const Attribute<Real>::Ptr mVdRef;
		/// Voltage q reference
		const Attribute<Real>::Ptr mVqRef;
		/// Flag for connection transformer usage
		Bool mWithConnectionTransformer=false;
		/// Flag for controller usage
		Bool mWithControl=true;

		/// Filter parameters
		Real mLf;
		Real mCf;
		Real mRf;
		Real mRc;

		// ### VSI Control parameters###
		Real mKiVoltageCtrl = 0;
		Real mKiCurrCtrl = 0;
		Real mKpVoltageCtrl = 0;
		Real mKpCurrCtrl = 0;
		Real mOmegaVSI;

		/// transformer
		Real mTransformerNominalVoltageEnd1;
		Real mTransformerNominalVoltageEnd2;
		Real mTransformerResistance;
		Real mTransformerInductance;
		Real mTransformerRatioAbs;
		Real mTransformerRatioPhase;
    public:
		explicit VSIVoltageSourceInverterDQ(Logger::Log Log, CPS::AttributeList::Ptr attributeList) :
			mLogger(Log),
			mOmegaN(attributeList->create<Real>("Omega_nom")),
			mThetaN(attributeList->create<Real>("Theta", 0)),
			mVdRef(attributeList->create<Real>("VdRef")),
			mVqRef(attributeList->create<Real>("VqRef")) { };

		/// Setter for general parameters of inverter
		void setParameters(Real sysOmega, Real VdRef, Real VqRef);
		/// Setter for filter parameters
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		/// Setter for optional connection transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
			Real ratioPhase, Real resistance, Real inductance);
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega);
		/// Setter for parameters of transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
			Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega);
		/// Setter for initial values applied in controllers
		void setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);
		/// 
		void withControl(Bool controlOn) { mWithControl = controlOn; };
		/// 
		void withConnectionTransformer(Bool withConnectionTransformer) { mWithConnectionTransformer = withConnectionTransformer; };
    };
}
}
