/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

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
				public MNASimPowerComp<Real>,
		public SharedFactory<VoltageSource> {
	protected:
		void updateVoltage(Real time);
	public:
		const Attribute<Complex>::Ptr mVoltageRef;
		const Attribute<Real>::Ptr mSrcFreq;
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

		void mnaPreStep(Real time, Int timeStepCount) override;
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;

		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;

		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
	};
}
}
}
