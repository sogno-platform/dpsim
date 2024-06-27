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
//#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/Solver/MNANonlinearVariableCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
 /// EMT Diode
class Diode :
	public MNASimPowerComp<Real>,
	public SharedFactory<Diode>,
    public MNANonlinearVariableCompInterface {
protected:
    ///TODO: mI_S and mV_T as const CPS::Attribute<Real>::Ptr, also change in pybind EMTComponents
    Real mI_S = 0.000001;
    Real mV_T = 0.027;
    Matrix Jacobian = Matrix::Zero(1, 1);
	Real itVoltage = 0.;
	Real itCurrent = 0.;
public:
	/// Defines UID, name, component parameters and logging level
	Diode(String uid, String name, Logger::Level logLevel = Logger::Level::off);
	/// Defines name, component parameters and logging level
	Diode(String name, Logger::Level logLevel = Logger::Level::off)
		: Diode(name, name, logLevel) { }

		// #### General ####
		///
		SimPowerComp<Real>::Ptr clone(String name);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

        void setParameters(Real, Real);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftSideVector);
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix);
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) { } //No right side vector stamps
		/// Update interface voltage from MNA system result
		void mnaCompUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaCompUpdateCurrent(const Matrix& leftVector);
		/// MNA pre and post step operations
		void mnaCompPreStep(Real time, Int timeStepCount) override; //No right side vector stamps
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// add MNA pre and post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

        virtual void iterationUpdate(const Matrix& leftVector) override;
        
        virtual bool hasParameterChanged() override {return true;}

        void calculateNonlinearFunctionResult();

        void updateJacobian();
	};
}
}
}
