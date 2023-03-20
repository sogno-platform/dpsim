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
#include <dpsim-models/DP/DP_Ph1_DPDQInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// @brief 4 Order Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	/// Modeling approach: thevenin prediction method
	class SynchronGenerator4OrderTPM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator4OrderTPM> {
	protected:
		/// Interface used to transform between DP and DQ vars
		DPDQInterface mDomainInterface;

		/// Constant part as conductance matrix
		Matrix mConductanceMatrixConst = Matrix::Zero(2,2);
		/// Varying part as resistance matrix
		Matrix mResistanceMatrixVarying = Matrix::Zero(2,2);

		// #### Model specific variables ####
		/// Transient emf
		const Attribute<Matrix>::Ptr mEdq_t;
		/// Original history voltage of VBR model
		Matrix mEh = Matrix::Zero(2,1);
		/// Modified history voltage of TPM model
        const Attribute<Complex>::Ptr mEhMod;

		// Variables saving values for later use
		/// Idp at k-1
		MatrixComp mIdpTwoPrevStep;
		/// Vdq at k
		Matrix mVdqPrevStep;
		/// Vdq at j-1
		Matrix mVdqPrevIter;
		/// Edq_t at k
		Matrix mEdqtPrevStep;

		/// A matrix of continuous time state space model
		Matrix mAStateSpace = Matrix::Zero(2,2);
		/// B matrix of continuous time state space model
		Matrix mBStateSpace = Matrix::Zero(2,2);
		/// C matrix of continuous time state space model
		Matrix mCStateSpace = Matrix::Zero(2,1);
	public:
		///
		SynchronGenerator4OrderTPM(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderTPM(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General Functions ####
		/// Initializes component from power flow data
		void specificInitialization() override;
		///
		void calculateStateSpaceMatrices();
		///
		void calculateConstantConductanceMatrix();
		///
		void stepInPerUnit() override;
		///
		void correctorStep() override;
		///
		void updateVoltage(const Matrix& leftVector) override;
		///
		bool requiresIteration() override;

		///
		void initializeResistanceMatrix() final {};

		// #### MNA Functions ####
		///
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
		///
		void mnaCompPostStep(const Matrix& leftVector) override;
		///
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;

		void setOperationalParametersPerUnit(Real nomPower,
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t);
	};
}
}
}
