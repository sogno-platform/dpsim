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
	class SynchronGenerator4OrderDCIM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public SharedFactory<SynchronGenerator4OrderDCIM> {
	protected:
		// ### State variables [p.u.]###
		/// voltage behing the transient reactance
		const Attribute<Matrix>::Ptr mEdq_t;
		/// 
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

		/// Transformation matrix dp->dq
		MatrixComp mDpToDq;

		/// Vector to create abc vector from a component
		MatrixComp mShiftVector;

		/// Park Transformation
		Matrix parkTransform(Real theta, const Matrix& abcVector);

		// #### General Functions ####
		/// DPecific component initialization
        void specificInitialization(); 
		///
		void stepInPerUnit();

		// ### MNA Section ###
        ///
        void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
        void mnaApplyRightSideVectorStamp(Matrix& rightVector);
        void mnaPostStep(const Matrix& leftVector);

	public:
		///
		SynchronGenerator4OrderDCIM(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderDCIM(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);
	};
}
}
}