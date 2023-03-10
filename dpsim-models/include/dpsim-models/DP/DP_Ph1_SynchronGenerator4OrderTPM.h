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
	/// Modeling approach: thevenin prediction method
	class SynchronGenerator4OrderTPM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public MNASyncGenInterface,
		public SharedFactory<SynchronGenerator4OrderTPM> {
	public:
		// Common elements of all VBR models
		/// voltage behind reactance phase a
        const Attribute<Complex>::Ptr mEvbr;

	protected:
		/// Constant part as conductance matrix
		Matrix mConductanceMatrixConst = Matrix::Zero(2,2);
		/// Varying part as resistance matrix
		Matrix mResistanceMatrixVarying = Matrix::Zero(2,2);

		/// Vector to create abc vector from a component
		MatrixComp mShiftVector;
		/// Matrix to convert Evbr from dq domain to abc domain (only phase a)
		MatrixComp mKvbr;

		// #### Model specific variables ####
		/// Transient emf
		const Attribute<Matrix>::Ptr mEdq_t;
		/// VBR voltage
		Matrix mEh_vbr;

		// Variables saving values for later use
		/// Idp at k-1
		MatrixComp mIdpTwoPrevStep;
		/// Idq at k
		Matrix mIdqPrevStep;
		/// Vdq at j-1
		Matrix mVdqPrevIter;
		/// Edq_t at k
		Matrix mEdqtPrevStep;
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
		void calculateAuxiliarConstants();
		///
		void calculateConductanceMatrix();
		///
		void stepInPerUnit() override;
		///
		void correctorStep() override;
		///
		void updateVoltage(const Matrix& leftVector) override;
		///
		bool requiresIteration() override;
		///
		void calculateAuxiliarVariables();
		///
		Matrix get_parkTransformMatrix();
		///
		void initializeResistanceMatrix() final {};

		// #### MNA Functions ####
		///
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
		///
		void mnaCompPostStep(const Matrix& leftVector) override;
		///
		void mnaCompApplySystemMatrixStamp(Matrix& systemMatrix) override;

		void setOperationalParametersPerUnit(Real nomPower,
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t);
	};
}
}
}
