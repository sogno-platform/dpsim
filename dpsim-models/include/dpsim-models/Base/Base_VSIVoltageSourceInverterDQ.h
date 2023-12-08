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
#include <dpsim-models/Base/VSIControlDQ.h>

namespace CPS {
namespace Base {
/// @brief Base model of average inverter
	class VSIVoltageSourceInverterDQ {
	private:
		/// Component logger
		Logger::Log mLogger;

	protected:
		// ### General Parameters ###
		/// Simulation step
		Real mTimeStep;
		/// Nominal Omega
		Real mOmegaN;

		// ### Inverter Parameters ###
		/// Nominal frequency
		Real mOmegaNom;
		/// Nominal voltage
		Real mVnom;
		/// Voltage d reference
		Real mVdRef;
		/// Voltage q reference
		Real mVqRef;
		/// Active power reference
		Real mPref;

		// ### Inverter Flags ###
		/// Flag for connection transformer usage
		Bool mWithConnectionTransformer=false;
		/// Flag for control droop usage
		Bool mWithDroop = false;

		// Filter parameters
		Real mLf;
		Real mCf;
		Real mRf;
		Real mRc;

		/// transformer
		Real mTransformerNominalVoltageEnd1;
		Real mTransformerNominalVoltageEnd2;
		Real mTransformerResistance;
		Real mTransformerInductance;
		Real mTransformerRatioAbs;
		Real mTransformerRatioPhase;

		// ### Inverter Variables ###
		/// Omega
		const Attribute<Real>::Ptr mOmega;
		/// System angle (rotating at nominal omega)
		const Attribute<Real>::Ptr mThetaSys;
		/// Inverter angle (rotating at inverter omega)
		const Attribute<Real>::Ptr mThetaInv;
		/// Measured voltage d-axis in local reference frame
		const Attribute<Real>::Ptr mVcap_d;
		/// Measured voltage q-axis in local reference frame
		const Attribute<Real>::Ptr mVcap_q;
		/// Measured current d-axis in local reference frame
		const Attribute<Real>::Ptr mIfilter_d;
		/// Measured current q-axis in local reference frame
		const Attribute<Real>::Ptr mIfilter_q;
		/// inverter terminal active power
		const Attribute<Real>::Ptr mActivePower;
		/// inverter terminal reactive power
		const Attribute<Real>::Ptr mReactivePower;
		/// Voltage as control output after transformation interface
		const Attribute<MatrixComp>::Ptr mVsref;

		// ### Voltage Controller Variables ###

		// #### Controllers ####
		/// Determines if VSI control is activated
		Bool mWithControl = true;
			
		/// Signal component modelling voltage regulator and exciter
		std::shared_ptr<Base::VSIControlDQ> mVSIController;
		

    public:
		explicit VSIVoltageSourceInverterDQ(Logger::Log Log, CPS::AttributeList::Ptr attributeList) :
			mLogger(Log),
			mOmega(attributeList->create<Real>("Omega", 0)),
			mThetaSys(attributeList->create<Real>("mThetaSys", 0)),
			mThetaInv(attributeList->create<Real>("mThetaInv", 0)),
			mVcap_d(attributeList->create<Real>("Vcap_d", 0)),
			mVcap_q(attributeList->create<Real>("Vcap_q", 0)),
			mIfilter_d(attributeList->create<Real>("Ifilter_d", 0)),
			mIfilter_q(attributeList->create<Real>("Ifilter_q", 0)),
			mActivePower(attributeList->create<Real>("P_elec", 0)),
			mReactivePower(attributeList->create<Real>("Q_elec", 0)),
			mVsref(attributeList->create<MatrixComp>("Vsref", MatrixComp::Zero(1,1))) { };

		/// Setter for general parameters of inverter
		void setParameters(Real sysOmega, Real VdRef, Real VqRef);
		/// Setter for filter parameters
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		/// Setter for optional connection transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
			Real ratioPhase, Real resistance, Real inductance);
		/// Setter for parameters of transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
			Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega);
		/// Setter for initial values applied in controllers
		void setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);
		/// 
		void withConnectionTransformer(Bool withConnectionTransformer) { mWithConnectionTransformer = withConnectionTransformer; };

		// ### Controllers ###
		/// Add VSI Controller
		void addVSIController(std::shared_ptr<Base::VSIControlDQ> VSIController);

	protected:
		void initializeControllerStates();
    };
}
}
