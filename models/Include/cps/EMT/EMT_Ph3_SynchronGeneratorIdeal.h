/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/EMT/EMT_Ph3_VoltageSource.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// Ideal voltage source representing a synchronous generator
	class SynchronGeneratorIdeal :
		public SimPowerComp<Real>,
		public MNAInterface,
		public SharedFactory<SynchronGeneratorIdeal> {
	private:
		/// Inner voltage source that represents the generator
		std::shared_ptr<EMT::Ph3::VoltageSource> mSubVoltageSource;
	public:
		/// Defines UID, name, component parameters and logging level
		SynchronGeneratorIdeal(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		SynchronGeneratorIdeal(String name,
			Logger::Level logLevel = Logger::Level::off);

		SimPowerComp<Real>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
	};
}
}
}
