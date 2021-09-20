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
#include <cps/Base/Base_SynchronGenerator.h>

namespace CPS {
namespace DP {
namespace Ph3 {
	/// Synchronous generator model in dq-reference frame
	class SynchronGeneratorDQ :
		public Base::SynchronGenerator,
		public MNAInterface,
		public SimPowerComp<Complex> {
	protected:
		/// Compensation current source set point
		MatrixComp mCompensationCurrent;

		/// Defines UID, name and logging level
		SynchronGeneratorDQ(String name, String uid, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		SynchronGeneratorDQ(String name, Logger::Level logLevel = Logger::Level::off);

		/// @brief Park transform as described in Krause
		///
		/// Balanced case because the zero sequence variable is ignored
		Matrix abcToDq0Transform(Real theta, MatrixComp& abc);

		/// @brief Inverse Park transform as described in Krause
		///
		/// Balanced case because the zero sequence variable is ignored
		MatrixComp dq0ToAbcTransform(Real theta, Matrix& dq0);

		// #### Deprecated ###
		/// calculate flux states using trapezoidal rule - depcrecated
		void trapezoidalFluxStates();
		// calculate current states using trapezoidal rule - depcrecated
		void trapezoidalCurrentStates();

	public:
		virtual ~SynchronGeneratorDQ();

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		void setParametersFundamentalPerUnit(Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia,
			Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle,
			Real initFieldVoltage, Real initMechPower);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
		/// Function parameters have to be given in Real units.
		void initialize(Real omega, Real timeStep);
		///
		void initialize(Matrix frequencies);
		///
		Real electricalTorque() { return mElecTorque * mBase_T; }
		///
		Real rotationalSpeed() { return mOmMech * mBase_OmMech; }
		///
		Real rotorPosition() { return mThetaMech; }
		/// Calculates flux and current from the voltage vector.
		void step(MatrixComp& voltage, Real time);

		// #### MNA Functions ####
		/// Initializes variables of component
		virtual void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr) = 0;
		/// Performs with the model of a synchronous generator
		/// to calculate the flux and current from the voltage vector.
		void mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time);
		///
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		///
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);

		/// Retrieves calculated voltage from simulation for next step
		virtual void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SynchronGeneratorDQ& synGen, Attribute<Matrix>::Ptr leftVector)
			: Task(synGen.mName + ".MnaPostStep"), mSynGen(synGen), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(synGen.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGeneratorDQ& mSynGen;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
