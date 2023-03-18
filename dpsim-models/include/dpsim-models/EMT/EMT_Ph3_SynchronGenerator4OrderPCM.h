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
namespace EMT {
namespace Ph3 {
	/// @brief 4 Order Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	class SynchronGenerator4OrderPCM :
		public Base::ReducedOrderSynchronGenerator<Real>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator4OrderPCM> {
	protected:


		/// sim flags
		bool mVoltageForm;

		// #### Model specific variables ####
		///
		Matrix mVdq0_prev;
		/// previous voltage behind the transient impedance (p.u.)
		const Attribute<Matrix>::Ptr mEdq0_t;
		Matrix mEdq0_t_pred;
		Matrix mEdq0_t_corr;
		/// derivative voltage behind the transient impedance (p.u.)
		Matrix mdEdq0_t;
		Matrix mdEdq0_t_corr;
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

	public:
		///
		SynchronGenerator4OrderPCM(const String& uid, const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderPCM(const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Real>::Ptr clone(const String& name);

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
		bool requiresIteration();
		///
		Matrix parkTransform(Real theta, const Matrix& abcVector);
		///
		Matrix inverseParkTransform(Real theta, const Matrix& dq0Vector);
		///
		void initializeResistanceMatrix() final {};


		/// Setters
		///
		void useVoltageForm(bool state) {mVoltageForm = state;}

		// #### MNA Functions ####
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaCompPostStep(const Matrix& leftVector);
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix){};
	};
}
}
}
