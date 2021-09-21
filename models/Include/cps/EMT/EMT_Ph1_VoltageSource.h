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

namespace CPS {
namespace EMT {
namespace Ph1 {
	/// \brief Ideal Voltage source model
	///
	/// This model uses modified nodal analysis to represent an ideal voltage source.
	/// For a voltage source between nodes j and k, a new variable (current across the voltage source)
	/// is added to the left side vector
	/// as unkown and it is taken into account for the equation of node j as positve and for the equation
	/// of node k as negative. Moreover
	/// a new equation ej - ek = V is added to the problem.
	class VoltageSource :
		public MNAInterface,
		public SimPowerComp<Real>,
		public SharedFactory<VoltageSource> {
	protected:
		void updateVoltage(Real time);

		Attribute<Complex>::Ptr mVoltageRef;
		Attribute<Real>::Ptr mSrcFreq;
	public:
		/// Defines UID, name and logging level
		VoltageSource(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		VoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
			: VoltageSource(name, name, logLevel) { }

		void setParameters(Complex voltageRef, Real srcFreq = -1);

		SimPowerComp<Real>::Ptr clone(String name);
		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) { }

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(VoltageSource& voltageSource) :
				Task(voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
				mAttributeDependencies.push_back(voltageSource.attribute("V_ref"));
				mModifiedAttributes.push_back(mVoltageSource.attribute("right_vector"));
				mModifiedAttributes.push_back(mVoltageSource.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			VoltageSource& mVoltageSource;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(VoltageSource& voltageSource, Attribute<Matrix>::Ptr leftVector) :
				Task(voltageSource.mName + ".MnaPostStep"), mVoltageSource(voltageSource), mLeftVector(leftVector)
			{
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mVoltageSource.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			VoltageSource& mVoltageSource;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
