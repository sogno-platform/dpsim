/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// Static phasor ResIndSeries model (only implemented for dynamic simulations!)
	class ResIndSeries :
		public MNASimPowerComp<Complex>,
		public MNATearInterface,
		public SharedFactory<ResIndSeries> {
	private:
		/// Impedance
		Complex mImpedance;
	public:
		/// Inductance [H]
		const Attribute<Real>::Ptr mInductance;
		///Resistance [ohm]
		const Attribute<Real>::Ptr mResistance;
		/// Defines UID, name and logging level
		ResIndSeries(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		ResIndSeries(String name, Logger::Level logLevel = Logger::Level::off)
			: ResIndSeries(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Sets model specific parameters
		void setParameters(Real resistance, Real inductance);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) override;


		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
		/// Update interface voltage from MNA system result
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// Update interface current from MNA system result
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// MNA pre and post step operations
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// add MNA pre and post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		// #### MNA Tear Section ####
		void mnaTearApplyMatrixStamp(SparseMatrixRow& tearMatrix) override;
	};
}
}
}
