/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Solver/MNASyncGenInterface.h>
#include <dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// @brief 6 Order Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	class SynchronGenerator6OrderIter :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator6OrderIter> {
	protected:
		/// sim flags
		bool mVoltageForm;

		// #### Model specific variables ####
		/// generator terminal at time k-1
		Matrix mVdq_prev;
		/// predicted generator current at time k
		Matrix mIdq_pred;
		/// corrected generator current at time k
		Matrix mIdq_corr;
		/// voltage behind the transient impedance at time k-1
		const Attribute<Matrix>::Ptr mEdq_t;
		/// predicted voltage behind the transient and subtransient impedance at time k+1
		Matrix mEdq_pred;
		/// corrected voltage behind the transient and subtransient impedance at time k+1
		Matrix mEdq_corr;
		/// corrected electrical torque
		Real mElecTorque_corr;
		/// predicted mechanical omega at time k
		Real mOmMech_pred;
		/// corrected mechanical omega at time k
		Real mOmMech_corr;
		/// prediction mechanical system angle at time k
		Real mThetaMech_pred;
		/// corrected mechanical system angle at time k
		Real mThetaMech_corr;
		/// predicted delta at time k
		Real mDelta_pred;
		/// corrected delta at time k
		Real mDelta_corr;

		/// State Matrix backward euler: Edq(k) = A * Edq(k) + B * Idq + C * Ef
		/// State Matrix trapezoidal rule (corrector step): x(k+1) = mA_prev * Edq(k-1) + mA_corr * Edq_corr(k) + B_corr * Idq_corr(k) + C * Ef
		/// A Matrix
		Matrix mA;
		Matrix mA_prev;
		Matrix mA_corr;
		/// B Matrix
		Matrix mB;
		Matrix mB_corr;
		/// Constant Matrix
		Matrix mC;

		/// Transformation matrix dp->dq
		MatrixComp mDpToDq;

		/// Vector to create abc vector from a component
		MatrixComp mShiftVector;

	public:
		///
		SynchronGenerator6OrderIter(const String& uid, const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator6OrderIter(const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(const String& name);

		// #### General Functions ####
		///
		void specificInitialization();
		///
		void calculateStateMatrix();
		///
		void stepInPerUnit();
		// 
		void correctorStep();
		/// 
		void updateVoltage(const Matrix& leftVector);
		///
		bool checkVoltageDifference();
		///
		Matrix parkTransform(Real theta, const Matrix& abcVector);


		/// Setters
		///
		void useVoltageForm(bool state) {mVoltageForm = state;}	

		// #### MNA Functions ####		
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaPostStep(const Matrix& leftVector);
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix){};
	};
}
}
}
