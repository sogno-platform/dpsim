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
	/// Modeling approach: delayed current injection + predictor corrector method
	class SynchronGenerator4OrderPCM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator4OrderPCM> {
	public:
		///
		SynchronGenerator4OrderPCM(const String& uid, const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderPCM(const String& name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(const String& name);

		// #### General Functions ####
		///
		void specificInitialization() override;
		///
		void calculateStateMatrix();
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
		void mnaCompApplySystemMatrixStamp(Matrix& systemMatrix) final {};

		///
		Matrix mEdqtPrevStep;
		Matrix mIdqPrevStep;
		Real mElecTorquePrevStep;
		Real mOmMechPrevStep;
		Real mThetaMechPrevStep;
		Real mDeltaPrevStep;

		///
		Matrix mVdqPrevIter;
	};
}
}
}
