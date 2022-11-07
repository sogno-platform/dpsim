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
	/// @brief 4 Order Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	class SynchronGenerator4OrderIter :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator4OrderIter> {
	protected:
		/// sim flags
		bool mVoltageForm;

		// #### Model specific variables ####
		///
		Matrix mVdq_prev;
		/// previous voltage behind the transient impedance (p.u.)
		Matrix mEdq_t;
		Matrix mEdq_t_pred;
		Matrix mEdq_t_corr;
		/// derivative voltage behind the transient impedance (p.u.)
		Matrix mdEdq_t;
		Matrix mdEdq_t_corr;
		///
		Real mElecTorque_corr;
		///
		Real mdOmMech = 0; 
		Real mdOmMech_corr = 0;
		Real mOmMech_pred;
		Real mOmMech_corr;
		/// prediction of mechanical system angle
		Real mThetaMech_pred;
		Real mThetaMech_corr;
		///
		Real mDelta_pred;
		Real mDelta_corr;
		Real mdDelta = 0;
		Real mdDelta_corr = 0;

		/// State Matrix x(k+1) = Ax(k) + Bu(k) + C
		/// A Matrix
		Matrix mA;
		/// B Matrix
		Matrix mB;
		/// Constant Matrix
		Matrix mC;

		/// Transformation matrix dp->dq
		MatrixComp mDpToDq;

		/// Vector to create abc vector from a component
		MatrixComp mShiftVector;

	public:
		///
		SynchronGenerator4OrderIter(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderIter(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);

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
