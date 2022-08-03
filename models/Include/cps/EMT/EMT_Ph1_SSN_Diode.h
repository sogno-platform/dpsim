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
#include <cps/Solver/MNAVariableCompInterface.h>
#include <cps/Solver/SSNNonlinearCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
namespace SSN {
 /// EMT Diode
class Diode :
	public MNAInterface,
	public SimPowerComp<Real>,
	public SharedFactory<Diode>,
    public MNAVariableCompInterface,
    public SSNNonlinearCompInterface {
protected:
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
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftSideVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector) { }
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// MNA pre and post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// add MNA pre and post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

        virtual void ssnUpdate(const Matrix& leftVector) override;
        
        virtual bool hasParameterChanged() override {return true;}

        virtual void SSNcalculateFunctionResult();

        virtual void ssnUpdateJacobian();

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Diode& Diode, Attribute<Matrix>::Ptr leftSideVector) :
				Task(**Diode.mName + ".MnaPostStep"),
				mDiode(Diode), mLeftVector(leftSideVector) {
				mDiode.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mDiode.mnaPostStep(time, timeStepCount, mLeftVector); };
		private:
			Diode& mDiode;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
}