/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Base/Base_Ph1_Inductor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief resistor inductor series element
	class ResIndSeries :
		public MNATearInterface,
		public MNASimPowerComp<Complex>,
		public SharedFactory<ResIndSeries> {
	protected:
		/// DC equivalent current source for harmonics [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance for harmonics [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous current value for harmonics
		MatrixComp mPrevCurrFac;
	public:
		/// Inductance [H]
		const Attribute<Real>::Ptr mInductance;
		///Resistance [ohm]
		const Attribute<Real>::Ptr mResistance;
		/// Defines UID, name and log level
		ResIndSeries(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		ResIndSeries(String name, Logger::Level logLevel = Logger::Level::off)
			: ResIndSeries(name, name, logLevel) { }

		// #### General ####
		/// Sets model specific parameters
		void setParameters(Real resistance, Real inductance);
		/// Return new instance with the same parameters
		SimPowerComp<Complex>::Ptr clone(String name);
		/// Initializes state variables considering the number of frequencies
		void initialize(Matrix frequencies);
		/// Initializes states from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix);
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system results
		void mnaCompUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system results
		void mnaCompUpdateCurrent();

		void mnaCompPreStep(Real time, Int timeStepCount) override;
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;

		/// Add MNA pre step dependencies
		void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;

		/// Add MNA post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
	};
}
}
}
