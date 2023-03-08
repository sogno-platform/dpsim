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
#include <dpsim-models/Solver/PFSolverInterfaceBranch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// Static phasor resistor model
	class Resistor :
		public MNASimPowerComp<Complex>,
		public Base::Ph1::Resistor,
		public MNATearInterface,
		public SharedFactory<Resistor>,
		public PFSolverInterfaceBranch {

	private:
		/// base apparent power[VA]
		Real mBaseApparentPower;
		/// base impedance [ohm]
		Real mBaseImpedance;
		/// base admittance [S]
		Real mBaseAdmittance;
		/// base voltage [V]
		Real mBaseVoltage;
		/// base current [A]
		Real mBaseCurrent;

		/// resistance [pu]
		Real mResistancePerUnit;
		/// conductance [pu]
		Real mConductancePerUnit;


	public:
		/// Defines UID, name and logging level
		Resistor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		Resistor(String name, Logger::Level logLevel = Logger::Level::off)
			: Resistor(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) override;

		// #### Powerflow section ####
		/// Set base voltage
		void setBaseVoltage(Real baseVoltage);
		/// Initializes component from power flow data
		void calculatePerUnitParameters(Real baseApparentPower);
		/// Stamps admittance matrix
		void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y);

		// #### MNA section ####
		///
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
