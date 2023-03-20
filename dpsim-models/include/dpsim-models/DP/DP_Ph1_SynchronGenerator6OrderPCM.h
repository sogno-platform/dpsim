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
	/// @brief 6 Order Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	class SynchronGenerator6OrderPCM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator6OrderPCM> {
	public:
		///
		SynchronGenerator6OrderPCM(const String& uid, const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator6OrderPCM(const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(const String& name);

		// #### General Functions ####
		///
		void specificInitialization() override;
		///
		void calculateStateSpaceMatrices();
		///
		void stepInPerUnit() override;
		//
		void correctorStep() override;
		///
		void updateVoltage(const Matrix& leftVector) override;
		///
		bool requiresIteration() override;
		///
		void initializeResistanceMatrix() final {};

		// #### MNA Functions ####
		///
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) final;
		///
		void mnaCompPostStep(const Matrix& leftVector) final;
		///
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) final {};

	protected:
		/// Interface used to transform between DP and DQ vars
		DPDQInterface mDomainInterface;

		// #### Model specific variables ####
		/// Subransient emf
		const Attribute<Matrix>::Ptr mEdq_s;
		/// Transient emf
		const Attribute<Matrix>::Ptr mEdq_t;
		///
		Matrix mEdqts = Matrix::Zero(4,1);

		// Variables saving values for later use
		/// Edqts at k
		Matrix mEdqtsPrevStep;
		/// Vdq at j-1
		Matrix mVdqPrevIter;
		/// Idq at k
		Matrix mIdqPrevStep;

		/// A matrix of continuous time state space model
		Matrix mAStateSpace = Matrix::Zero(4,4);
		/// B matrix of continuous time state space model
		Matrix mBStateSpace = Matrix::Zero(4,2);
		/// C matrix of continuous time state space model
		Matrix mCStateSpace = Matrix::Zero(4,1);
	};
}
}
}
