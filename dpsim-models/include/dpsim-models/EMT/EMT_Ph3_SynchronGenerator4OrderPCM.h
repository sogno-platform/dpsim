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
	/// Modeling approach: delayed current injection + predictor corrector method
	class SynchronGenerator4OrderPCM :
		public Base::ReducedOrderSynchronGenerator<Real>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator4OrderPCM> {

	public:
		///
		SynchronGenerator4OrderPCM(const String& uid, const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderPCM(const String& name, Logger::Level logLevel = Logger::Level::off);

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
		Matrix parkTransform(Real theta, const Matrix& abcVector);
		///
		Matrix inverseParkTransform(Real theta, const Matrix& dq0Vector);
		///
		void initializeResistanceMatrix() final {};

		/// Warning if this method is applied: the model is exclusively implemented as current source and this setter will have no impact!
		void setModelAsNortonSource(Bool modelAsCurrentSource) override {
			SPDLOG_LOGGER_WARN(mSLog, "This model can exclusively be used as current source. The setter setModelAsNortonSource will have no impact on the model!");
		}

		// #### MNA Functions ####
		///
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) final;
		///
		void mnaCompPostStep(const Matrix& leftVector) final;
		///
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) final {};

	protected:
		// #### Model specific variables ####
		/// Transient emf
		const Attribute<Matrix>::Ptr mEdq0_t;

		// Variables saving values for later use
		/// Trapedzoidal based state space matrix Ad
		Matrix mAdTrapezoidal;
		/// Trapedzoidal based state space matrix Bd
		Matrix mBdTrapezoidal;
		/// Trapedzoidal based state space matrix Cd
		Matrix mCdTrapezoidal;
		/// Edqt at k
		Matrix mEdq0tPrevStep;
		/// Idq at k
		Matrix mIdq0PrevStep;
		/// Vdq at k
		Matrix mVdq0PrevStep;
		/// Vdq at j-1
		Matrix mVdq0PrevIter;

		/// A matrix of continuous time state space model
		Matrix mAStateSpace = Matrix::Zero(3,3);
		/// B matrix of continuous time state space model
		Matrix mBStateSpace = Matrix::Zero(3,3);
		/// C matrix of continuous time state space model
		Matrix mCStateSpace = Matrix::Zero(3,1);

	
	};
}
}
}
