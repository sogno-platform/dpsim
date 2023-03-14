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
namespace DP {
namespace Ph1 {
	/// @brief Delayed-Current-Injection (DCIM) implementation
	/// of 4th order synchronous generator model
	class SynchronGenerator6OrderDCIM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public SharedFactory<SynchronGenerator6OrderDCIM> {
	protected:
		// ### State variables [p.u.]###
		/// voltage behing the transient reactance
		const Attribute<Matrix>::Ptr mEdq_t;
		/// voltage behind subtransient reactance
		const Attribute<Matrix>::Ptr mEdq_s;
		///
		Matrix mStates;
		Matrix mStates_prev;

		/// Auxiliar VBR constants
		///
		Real mAd_t;
		///
		Real mBd_t;
		///
		Real mAq_t;
		///
		Real mBq_t;
		///
		Real mDq_t;
		///
		Real mAd_s;
		///
		Real mAq_s;
		///
		Real mBd_s;
		///
		Real mBq_s;
		///
		Real mCd_s;
		///
		Real mCq_s;

		/// state representation matrix
		///
		Matrix mA;
		///
		Matrix mA_inv;
		///
		Matrix mB;
		///
		Matrix mC;

		/// Vector to create abc vector from a component
		MatrixComp mShiftVector;

		/// Park Transformation
		Matrix parkTransform(Real theta, const Matrix& abcVector);

		// #### General Functions ####
		/// DPecific component initialization
        void specificInitialization();
		///
		void stepInPerUnit();
		///
		void initializeResistanceMatrix() final {};

		// ### MNA Section ###
        ///
        void mnaCompApplySystemMatrixStamp(Matrix& systemMatrix);
        void mnaCompApplyRightSideVectorStamp(Matrix& rightVector);
        void mnaCompPostStep(const Matrix& leftVector);

	public:
		///
		SynchronGenerator6OrderDCIM(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator6OrderDCIM(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);
	};
}
}
}
