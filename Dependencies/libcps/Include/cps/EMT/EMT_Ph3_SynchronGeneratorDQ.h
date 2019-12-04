/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <cps/PowerComponent.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Base/Base_SynchronGenerator.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// Synchronous generator model in dq-reference frame
	class SynchronGeneratorDQ :
		public Base::SynchronGenerator,
		public MNAInterface,
		public PowerComponent<Real> {
	protected:
		/// Compensation current source set point
		Matrix mCompensationCurrent;

		/// Defines UID, name and logging level
		SynchronGeneratorDQ(String name, String uid, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		SynchronGeneratorDQ(String name, Logger::Level logLevel = Logger::Level::off);

		/// @brief Park transform as described in Krause
		///
		/// Balanced case because the zero sequence variable is ignored
		Matrix abcToDq0Transform(Real theta, Matrix& abc);

		/// @brief Inverse Park transform as described in Krause
		///
		/// Balanced case because the zero sequence variable is ignored
		Matrix dq0ToAbcTransform(Real theta, Matrix& dq0);

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

		///
		void initialize(Matrix frequencies);
		///
		Real electricalTorque() { return mElecTorque * mBase_T; }
		///
		Real rotationalSpeed() { return mOmMech * mBase_OmMech; }
		///
		Real rotorPosition() { return mThetaMech; }

		/// General step function for standalone simulation
		void step(Matrix& voltage, Real time);

		// #### MNA Functions ####
		/// Initializes variables of component
		virtual void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr) = 0;
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
