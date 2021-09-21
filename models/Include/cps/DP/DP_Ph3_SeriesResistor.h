/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Definitions.h>
#include <cps/Logger.h>
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph3 {
	///
	class SeriesResistor :
		public Base::Ph1::Resistor,
		public MNAInterface,
		public SimPowerComp<Complex>,
		public SharedFactory<SeriesResistor> {

	public:
			/// Defines UID, name and logging level
		SeriesResistor(String uid, String name,	Logger::Level logLevel = Logger::Level::off);
			/// Defines name and logging level
		SeriesResistor(String name,	Logger::Level logLevel = Logger::Level::off)
			: SeriesResistor(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		// #### MNA section ####
		///
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		///
		void mnaUpdateVoltage(const Matrix& leftVector);
		///
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SeriesResistor& resistor, Attribute<Matrix>::Ptr leftSideVector) :
				Task(resistor.mName + ".MnaPostStep"), mResistor(resistor), mLeftVector(leftSideVector)
			{
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
				mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SeriesResistor& mResistor;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
