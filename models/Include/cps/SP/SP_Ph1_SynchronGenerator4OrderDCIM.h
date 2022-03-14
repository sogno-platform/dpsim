/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Base/Base_ReducedOrderSynchronGenerator.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// @brief Delayed-Current-Injection (DCIM) implementation
	/// of 4th order synchronous generator model
	class SynchronGenerator4OrderDCIM :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public SharedFactory<SynchronGenerator4OrderDCIM> {
	protected:
		// ### State variables [p.u.]###
		/// voltage behing the transient reactance
		Matrix mEdq_t;

		/// state representation matrix
		///
		Matrix mA;
		///
		Matrix mB;
		/// 
		Matrix mC;

		///
		void calculateStateMatrix();

		/// Park Transformation
		///
		Matrix mDqToComplexA;
		///
		Matrix mComplexAToDq;
		///
        Matrix get_DqToComplexATransformMatrix();

		// #### General Functions ####
		/// Specific component initialization
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