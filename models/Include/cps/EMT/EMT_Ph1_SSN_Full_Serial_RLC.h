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
#include <cps/Base/Base_Ph1_Inductor.h>
#include <cps/Base/Base_Ph1_Capacitor.h>
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
namespace SSN {
	class Full_Serial_RLC :
		public Base::Ph1::Inductor,
        public Base::Ph1::Capacitor,
        public Base::Ph1::Resistor,
		public MNAInterface,
		public SimPowerComp<Real>,
		public SharedFactory<Full_Serial_RLC> {
	protected:
        Matrix State = Matrix::Zero(2, 1);
        Matrix yHistory =  Matrix::Zero(1, 1);

		Matrix Dufour_u_n_t = Matrix::Zero(1, 1);

        Matrix Dufour_A_k_hat = Matrix::Zero(2, 2);
		Matrix Dufour_B_k_hat = Matrix::Zero(2, 1);
        Matrix Dufour_B_k_n_hat = Matrix::Zero(2, 1);
		Matrix Dufour_W_k_n = Matrix::Zero(1, 1);
        Matrix Dufour_C_k_n = Matrix(1, 2);
	public:
		/// Defines UID, name, component parameters and logging level
		Full_Serial_RLC(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		Full_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
			: Full_Serial_RLC(name, name, logLevel) { }

        void setParameters(Real resistance, Real inductance, Real capacitance);

		SimPowerComp<Real>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);

		void ssnUpdateState();
		bool isLinear() const
		{
			return true;
		}		

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Full_Serial_RLC& full_Serial_RLC) :
				Task(**full_Serial_RLC.mName + ".MnaPreStep"), mFull_Serial_RLC(full_Serial_RLC) {

				mModifiedAttributes.push_back(full_Serial_RLC.attribute("right_vector"));
				mPrevStepDependencies.push_back(full_Serial_RLC.attribute("i_intf"));
				mPrevStepDependencies.push_back(full_Serial_RLC.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Full_Serial_RLC& mFull_Serial_RLC;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Full_Serial_RLC& full_Serial_RLC, Attribute<Matrix>::Ptr leftVector) :
				Task(**full_Serial_RLC.mName + ".MnaPostStep"), mFull_Serial_RLC(full_Serial_RLC), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mFull_Serial_RLC.attribute("v_intf"));
				mModifiedAttributes.push_back(mFull_Serial_RLC.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Full_Serial_RLC& mFull_Serial_RLC;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
}