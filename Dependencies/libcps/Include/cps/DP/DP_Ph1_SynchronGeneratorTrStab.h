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
#include <cps/DP/DP_Ph1_VoltageSource.h>
#include <cps/DP/DP_Ph1_Inductor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// @brief Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	class SynchronGeneratorTrStab :
		public Base::SynchronGenerator,
		public MNAInterface,
		public PowerComponent<Complex>,
		public SharedFactory<SynchronGeneratorTrStab> {
	protected:
		// #### Model specific variables ####
		/// emf behind transient reactance
		Complex mEp;
		/// fixed absolute value of emf behind transient reactance
		Real mEp_abs;
		/// Angle by which the emf Ep is leading the system reference frame
		Real mDelta_p;
		/// Absolute d-axis transient reactance X'd
 		Real mXpd;
		/// Absolute d-axis transient inductance
		Real mLpd;
		/// Equivalent impedance for loadflow calculation
		Complex mImpedance;
		/// Inner voltage source that represents the generator
		std::shared_ptr<VoltageSource> mSubVoltageSource;
		/// Inner inductor that represents the generator impedance
		std::shared_ptr<Inductor> mSubInductor;
		// Logging
		Matrix mStates;
	public:
		///
		SynchronGeneratorTrStab(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGeneratorTrStab(String name, Logger::Level logLevel = Logger::Level::off)
			: SynchronGeneratorTrStab(name, name, logLevel) { }

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General Functions ####
		///
		void setInitialValues(Complex elecPower, Real mechPower);
		/// \brief Initializes the machine parameters
		void setFundamentalParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
			Real Ll, Real Lmd, Real Llfd, Real inertia);
		/// \brief Initializes the machine parameters
		void setStandardParametersSI(Real nomPower, Real nomVolt, Real nomFreq, Int polePairNumber,
			Real Rs, Real Lpd, Real inertiaJ, Real Kd = 0);
		/// \brief Initializes the machine parameters
		void setStandardParametersPU(Real nomPower, Real nomVolt, Real nomFreq, Real Xpd, Real inertia);
		/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
		/// Function parameters have to be given in Real units.
		void initialize(Real omega, Real timeStep);
		///
		void step(Real time);
		///
		void initialize(Matrix frequencies);
		///
		void initializeFromPowerflow(Real frequency);

		// #### MNA Functions ####
		/// Initializes variables of component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Performs with the model of a synchronous generator
		/// to calculate the flux and current from the voltage vector.
		void mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time);
		///
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		///
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Retrieves calculated voltage from simulation for next step
		void mnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time);
		///
		void mnaUpdateCurrent(const Matrix& leftVector) { }
		///
		void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(SynchronGeneratorTrStab& generator) :
				Task(generator.mName + ".MnaPreStep"), mGenerator(generator) {
				// other attributes generally also influence the pre step,
				// but aren't marked as writable anyway
				mPrevStepDependencies.push_back(generator.attribute("v_intf"));
				mModifiedAttributes.push_back(generator.mSubVoltageSource->attribute("V_ref"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGeneratorTrStab& mGenerator;
		};

		class AddBStep : public Task {
		public:
			AddBStep(SynchronGeneratorTrStab& generator) :
				Task(generator.mName + ".AddBStep"), mGenerator(generator) {
				mAttributeDependencies.push_back(generator.mSubVoltageSource->attribute("right_vector"));
				mAttributeDependencies.push_back(generator.mSubInductor->attribute("right_vector"));
				mModifiedAttributes.push_back(generator.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGeneratorTrStab& mGenerator;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SynchronGeneratorTrStab& generator, Attribute<Matrix>::Ptr leftVector) :
				Task(generator.mName + ".MnaPostStep"), mGenerator(generator), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				mModifiedAttributes.push_back(generator.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGeneratorTrStab& mGenerator;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
