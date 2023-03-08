/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// @brief Delayed-Current-Injection (DCIM) implementation
	/// of 4th order synchronous generator model
	class SynchronGenerator4OrderDCIM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public SharedFactory<SynchronGenerator4OrderDCIM> {

	public:
		// ### State variables [p.u.]###
		/// voltage behing the transient reactance
		const Attribute<Matrix>::Ptr mEdq_t;

	protected:
		Matrix mStates;
		Matrix mStates_prev;

		/// state representation matrix
		///
		Real mAd;
		///
		Real mBd;
		///
		Real mAq;
		///
		Real mBq;
		///
		Real mCq;
		///
		Matrix mA;
		///
		Matrix mA_inv;
		///
		Matrix mB;
		///
		Matrix mC;
		///

		/// Park Transformation
		///
		Matrix mDqToComplexA;
		///
		Matrix mComplexAToDq;
		///
        Matrix get_DqToComplexATransformMatrix();

		// #### General Functions ####
		/// Specific component initialization
        void specificInitialization() final;
		///
		void initializeResistanceMatrix() final {};
		///
		void stepInPerUnit() final;
		///
		void calculateStateMatrix();

		// ### MNA Section ###
        ///
        void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
        void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
        void mnaCompPostStep(const Matrix& leftVector) override;

	public:
		///
		SynchronGenerator4OrderDCIM(const String & uid, const String & name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderDCIM(const String & name, Logger::Level logLevel = Logger::Level::off);
		/// DCIM is only implmented as current source!
		void setModelAsCurrentSource(Bool modelAsCurrentSource) const {
			SPDLOG_LOGGER_DEBUG(mSLog, "DCIM model can only be used as current source!");
		}
	};
}
}
}
