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
		public SimPowerComp<Complex>,
		public SharedFactory<SynchronGeneratorTrStab> {
	protected:
		// #### Model specific variables ####
		/// emf behind transient reactance
		Complex mEp;
		/// fixed absolute value of emf behind transient reactance
		Real mEp_abs;
		/// Angle by which the emf Ep is leading the system reference frame
		Real mEp_phase;
		/// Angle by which the emf Ep is leading the terminal voltage
		Real mDelta_p;
		/// Absolute d-axis transient reactance X'd
 		Real mXpd;
		/// Absolute d-axis transient inductance
		Real mLpd;
		/// Absolute damping coefficient
		Real mKd;
		/// Equivalent impedance for loadflow calculation
		Complex mImpedance;
		/// Inner voltage source that represents the generator
		std::shared_ptr<VoltageSource> mSubVoltageSource;
		/// Inner inductor that represents the generator impedance
		std::shared_ptr<Inductor> mSubInductor;
		/// Logging
		Matrix mStates;
		/// Flag for usage of attribute of w_ref (otherwise mNomOmega is used)
		Bool mUseOmegaRef = false;
		/// Flag for usage of actual mechanical speed for torque conversion (otherwise mNomOmega is used)
		Bool mConvertWithOmegaMech = true;
	public:
		///
		SynchronGeneratorTrStab(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGeneratorTrStab(String name, Logger::Level logLevel = Logger::Level::off)
			: SynchronGeneratorTrStab(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General Functions ####
		/// Flags to modify model behavior
		void setModelFlags(Bool useOmegaRef, Bool convertWithOmegaMech);
		///
		void setInitialValues(Complex elecPower, Real mechPower);
		/// \brief Initializes the machine parameters
		void setFundamentalParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
			Real Ll, Real Lmd, Real Llfd, Real inertia, Real D=0);
		/// \brief Initializes the machine parameters
		void setStandardParametersSI(Real nomPower, Real nomVolt, Real nomFreq, Int polePairNumber,
			Real Rs, Real Lpd, Real inertiaJ, Real Kd = 0);
		/// \brief Initializes the machine parameters
		void setStandardParametersPU(Real nomPower, Real nomVolt, Real nomFreq, Real Xpd, Real inertia,
			Real Rs=0, Real D=0);
		///
		void step(Real time);
		///
		void initializeFromNodesAndTerminals(Real frequency);

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
		void mnaUpdateCurrent(const Matrix& leftVector);
		///
		void mnaUpdateVoltage(const Matrix& leftVector);

		void setReferenceOmega(Attribute<Real>::Ptr refOmegaPtr, Attribute<Real>::Ptr refDeltaPtr);

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
				mAttributeDependencies.push_back(generator.mSubInductor->attribute("i_intf"));
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
