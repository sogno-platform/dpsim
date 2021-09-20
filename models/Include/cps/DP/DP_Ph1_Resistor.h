/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNATearInterface.h>
#include <cps/Solver/DAEInterface.h>
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief Dynamic phasor resistor model
	class Resistor :
		public Base::Ph1::Resistor,
		public MNATearInterface,
		public DAEInterface,
		public SimPowerComp<Complex>,
		public SharedFactory<Resistor> {
	public:
		/// Defines UID, name and logging level
		Resistor(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		/// Defines name and logging level
		Resistor(String name, Logger::Level logLevel = Logger::Level::off)
			: Resistor(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps system matrix considering the frequency index
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		void mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);
		void mnaUpdateCurrentHarm();
		/// MNA pre and post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// add MNA pre and post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Resistor& resistor, Attribute<Matrix>::Ptr leftSideVector) :
				Task(resistor.mName + ".MnaPostStep"),
				mResistor(resistor), mLeftVector(leftSideVector) {
				mResistor.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mResistor.mnaPostStep(time, timeStepCount, mLeftVector); };
		private:
			Resistor& mResistor;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		class MnaPostStepHarm : public Task {
		public:
			MnaPostStepHarm(Resistor& resistor, std::vector<Attribute<Matrix>::Ptr> &leftVectors) :
				Task(resistor.mName + ".MnaPostStepHarm"),
				mResistor(resistor), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
				mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Resistor& mResistor;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};

		// #### MNA Tear Section ####
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);

		// #### DAE Section ####
		///Residual Function for DAE Solver
		void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off);
		///Voltage Getter
		Complex daeInitialize();
	};
}
}
}
