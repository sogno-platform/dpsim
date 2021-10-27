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
#include <cps/Solver/PFSolverInterfaceBranch.h>
#include <cps/Definitions.h>
#include <cps/Logger.h>
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// Static phasor resistor model
	class Resistor :
		public Base::Ph1::Resistor,
		public MNATearInterface,
		public SimPowerComp<Complex>,
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
		void initializeFromNodesAndTerminals(Real frequency);

		// #### Powerflow section ####
		/// Set base voltage
		void setBaseVoltage(Real baseVoltage);
		/// Initializes component from power flow data
		void calculatePerUnitParameters(Real baseApparentPower);
		/// Stamps admittance matrix
		void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y);

		// #### MNA section ####
		///
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);
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
		// #### MNA Tear Section ####
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);

	};
}
}
}
