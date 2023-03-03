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
#include <dpsim-models/Base/Base_Ph3_Resistor.h>
namespace CPS {
namespace EMT {
namespace Ph3 {
 /// EMT Resistor
class Resistor :
	public MNASimPowerComp<Real>,
	public Base::Ph3::Resistor,
	public SharedFactory<Resistor> {
protected:
public:
	/// Defines UID, name, component parameters and logging level
	Resistor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
	/// Defines name, component parameters and logging level
	Resistor(String name, Logger::Level logLevel = Logger::Level::off)
		: Resistor(name, name, logLevel) { }

		// #### General ####
		///
		SimPowerComp<Real>::Ptr clone(String name);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// enable DP to EMT bach transformation
		void enableBackShift();


		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftSideVector) override;
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override { }
		/// Update interface voltage from MNA system result
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// Update interface current from MNA system result
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// MNA pre and post step operations
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// add MNA pre and post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
	};
}
}
}
