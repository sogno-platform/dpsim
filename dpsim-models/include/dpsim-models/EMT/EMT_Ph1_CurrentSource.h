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
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Base/Base_Ph1_CurrentSource.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
	/// \brief Ideal current source model
	///
	/// A positive current is flowing out of
	/// node1 and into node2.
	class CurrentSource :
		public MNASimPowerComp<Real>,
		public DAEInterface,
		public SharedFactory<CurrentSource> {
	public:
		const Attribute<Complex>::Ptr mCurrentRef;
		const Attribute<Real>::Ptr mSrcFreq;

		/// Defines UID, name and logging level
		CurrentSource(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		///
		CurrentSource(String name, Logger::Level logLevel = Logger::Level::off)
			: CurrentSource(name, name, logLevel) { }

		SimPowerComp<Real>::Ptr clone(String name);

		void setParameters(Complex currentRef, Real srcFreq = -1);
		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) { }

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) { }
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector);
		///
		void mnaCompUpdateVoltage(const Matrix& leftVector);

		void updateState(Real time);

		void mnaCompPreStep(Real time, Int timeStepCount) override;
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;

		/// Add MNA pre step dependencies
		void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;

		/// Add MNA post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		///
		void daeInitialize(double time, double state[], double dstate_dt[],
			double absoluteTolerances[], double stateVarTypes[], int& offset) override;
		/// Residual function for DAE Solver
		void daeResidual(double time, const double state[], const double dstate_dt[], 
			double resid[], std::vector<int>& off) override;
		/// Calculation of jacobian
		void daeJacobian(double current_time, const double state[], const double dstate_dt[], 
			SUNMatrix jacobian, double cj, std::vector<int>& off) override {};
		///
		void daePostStep(double Nexttime, const double state[], 
			const double dstate_dt[], int& offset) override;
		///
		int getNumberOfStateVariables() override {return 0;}
	};
}
}
}
