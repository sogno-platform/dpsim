/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Base/Base_SimpSynchronousGenerator.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// @brief Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	class SynchronGenerator4OrderDQ :
		public Base::SimpSynchronousGenerator<Complex>,
		public SharedFactory<SynchronGenerator4OrderDQ> {
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
		SynchronGenerator4OrderDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderDQ(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);
	};
}
}
}