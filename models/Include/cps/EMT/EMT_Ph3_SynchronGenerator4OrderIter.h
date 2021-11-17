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
#include <cps/Solver/MNASyncGenInterface.h>
#include <cps/Base/Base_SimpSynchronousGenerator.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// @brief 4 Order Synchronous generator model for transient stability analysis
	///
	/// This model is based on Eremia section 2.1.6.
	class SynchronGenerator4OrderIter :
		public SimPowerComp<Real>,
		public MNAInterface,
		public MNASyncGenInterface,
		public Base::SimpSynchronousGenerator,
		public SharedFactory<SynchronGenerator4OrderIter> {
	protected:
		///
		Real mTimeStep;
		Real mSimTime;
		Int mStepNumber;
		Int mNumIterations2;
		Real mFaultClearingTime;
		Real mFaultTime;

		/// sim flags
		NumericalMethod mNumericalMethod;
		bool mVoltageForm;

		// #### Model specific variables ####
		

		/// Variables (p.u.)
		/// dq stator terminal voltage (p.u.)
		/// (0,0) = Vd
		/// (1,0) = Vq
		/// (2,0) = V0
		Matrix mVdq0;
		Matrix mVdq0_prev;
		/// dq armature current (p.u.)
		/// (0,0) = Id
		/// (1,0) = Iq
		/// (2,0) = I0
		Matrix mIdq0;
		Matrix mIdq0_preFault;
		/// voltage behind the transient impedance (p.u.)
		/// (0,0) = Eq
		/// (1,0) = Ep
		Matrix mEdq0_t;
		/// previous voltage behind the transient impedance (p.u.)
		Matrix mEdq0_t_prev;
		/// derivative voltage behind the transient impedance (p.u.)
		/// (0,0) = Eq
		/// (1,0) = Ep
		/// derivative of the voltage behind the transient impedance (p.u.)
		Matrix mdEdq0_t;
		Matrix mdEdq0_t_prev;

		///
		Real mOmMech_prev;
		/// derivative of rotor speed
		Real mdOmMech;
		Real mdOmMech0;
		/// mechanical system angle
		Real mThetaMech;
		/// prediction of mechanical system angle
		Real mThetaMech_pred;
		/// derivative of mechanical system angle
		Real mdThetaMech;
		/// Load angle
		Real mDelta;
		Real mDelta_prev;
		///
		Real mdDelta;
		Real mdDelta0;

		/// State Matrix x(k+1) = Ax(k) + Bu(k) + C
		/// A Matrix
		Matrix mA;
		/// B Matrix
		Matrix mB;
		/// Constant Matrix
		Matrix mC;

	public:
		///
		SynchronGenerator4OrderIter(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderIter(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Real>::Ptr clone(String name);

		// #### General Functions ####
		///
		void initializeFromNodesAndTerminals(Real frequency);
		///
		void initialize();
		///
		void calculateStateMatrix();
		///
		void stepInPerUnit();
		// 
		bool step();
		/// 
		void updateVoltage(const Matrix& leftVector);
		///
		bool checkVoltageDifference();
		///
		Matrix parkTransform(Real theta, const Matrix& abcVector);
		///
		Matrix inverseParkTransform(Real theta, const Matrix& dq0Vector);


		/// Setters
		///
		void useVoltageForm(bool state) {mVoltageForm = state;}	
		///
		void setNumericalMethod(NumericalMethod numericalMethod) {mNumericalMethod = numericalMethod;}
		///
		void setFaultTime(Real faultTime) {mFaultTime = faultTime;}
		///
		void setFaultClearingTime(Real faultClearingTime) {mFaultClearingTime = faultClearingTime;}

		// #### MNA Functions ####		
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaPostStep(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(SynchronGenerator4OrderIter& synGen)
				: Task(synGen.mName + ".MnaPreStep"), mSynGen(synGen) {
				mModifiedAttributes.push_back(synGen.attribute("right_vector"));
				mPrevStepDependencies.push_back(synGen.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGenerator4OrderIter& mSynGen;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SynchronGenerator4OrderIter& synGen, Attribute<Matrix>::Ptr leftSideVector) :
				Task(synGen.mName + ".MnaPostStep"),
				mSynGen(synGen), mLeftVector(leftSideVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(synGen.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			SynchronGenerator4OrderIter& mSynGen;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
}
}
