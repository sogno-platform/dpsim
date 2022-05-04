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

namespace CPS {
namespace Base {

	template <typename VarType>
	class ReducedOrderSynchronGenerator : 
		public SimPowerComp<VarType>,
		public MNAInterface {

		protected:
			///
			ReducedOrderSynchronGenerator(String uid, String name, Logger::Level logLevel);
			///
			void initializeFromNodesAndTerminals(Real frequency);
			/// Function to initialize the specific variables of each SG model
			virtual void specificInitialization()=0;
			///
        	virtual void stepInPerUnit()=0;
			
			// ### MNA Section ###
        	///
        	void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
        	virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix)=0;
        	virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector)=0;
        	virtual void mnaPostStep(const Matrix& leftVector)=0;

			///
			Real mTimeStep;
			Real mSimTime;

			// ### State variables [p.u.]###
			/// dq stator terminal voltage
			/// (0,0) = Vd
			/// (1,0) = Vq
			Matrix mVdq;
			/// dq armature current
			/// (0,0) = Id
			/// (1,0) = Iq
			Matrix mIdq;
			/// dq stator terminal voltage
			/// (0,0) = Vd
			/// (1,0) = Vq
			/// (2,0) = V0
			Matrix mVdq0;
			/// dq armature current
			/// (0,0) = Id
			/// (1,0) = Iq
			/// (2,0) = I0
			Matrix mIdq0;

			// ### Base quantities (stator refered) ###
			/// Nominal power
			Real mNomPower;
			/// Nominal voltage
			Real mNomVolt;
			// Nominal frequency
			Real mNomFreq;
			/// Nominal Omega
			Real mNomOmega;
			/// Base voltage RMS
			Real mBase_V_RMS;
			/// Base peak voltage
			Real mBase_V;
			/// Base RMS current
			Real mBase_I_RMS;
			/// Base peak current
			Real mBase_I;
			/// Base impedance
			Real mBase_Z;
			/// Base omega electric
			Real mBase_OmElec;
			/// Base inductance
			Real mBase_L;
			/// Base omega mech
			Real mBase_OmMech;
			/// Inertia 
			Real mH;

			// ### Operational Parameters  (p.u.) ###
			/// d-axis inductance
			Real mLd = 0;
			/// d-axis inductance
			Real mLq = 0;
			/// 0-axis inductance
			Real mL0 = 0;
			/// Subtransient d-axis inductance
			Real mLd_t = 0;
			/// Subtransient q-axis inductance
			Real mLq_t = 0; 
			/// Subtransient d-axis inductance
			Real mLd_s = 0;
			/// Subtransient q-axis inductance
			Real mLq_s = 0;
			/// Transient time constant of d-axis
			Real mTd0_t = 0;
			/// Transient time constant of q-axis
			Real mTq0_t = 0;
			/// Subtransient time constant of d-axis
			Real mTd0_s = 0;
			/// Subtransient time constant of q-axis
			Real mTq0_s = 0;
			/// d-axis additional leakage time constant
			Real mTaa = 0;

			// ### Initial values ###
			/// Complex interface current
			Complex mIntfCurrentComplex;
			/// Complex interface voltage
			Complex mIntfVoltageComplex;
			/// initial electrical power
			Complex mInitElecPower;
			/// initial mechanical power 
			Real mInitMechPower;
			/// initial terminal voltage phase a (p.u.)
			Complex mInitVoltage;
			/// angle of initial armature voltage
			Real mInitVoltageAngle;
			/// initial armature voltage phase a (p.u.)
			Complex mInitCurrent;
			/// angle of initial armature current
			Real mInitCurrentAngle;
			/// initial field voltage (p.u.)
			Real mEf;


			// ### State variables [p.u.]###
			/// stator electrical torque
			Real mElecTorque;
			/// Mechanical torque
			Real mMechTorque;
			/// Rotor speed
			Real mOmMech;
			/// mechanical system angle (between d-axis and stator a-axis)
			Real mThetaMech;
			/// Load angle (between q-axis and stator a-axis)
			Real mDelta;		

			/// Flag to remember when initial values are set
			Bool mInitialValuesSet = false;

		public:
			/// Destructor - does nothing.
			virtual ~ReducedOrderSynchronGenerator() { }

			/// 
			void setBaseParameters(Real nomPower, Real nomVolt, Real nomFreq);
			/// Initialization for 3 Order SynGen
			void setOperationalParametersPerUnit(Real nomPower, 
				Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
				Real Ld_t, Real Td0_t);
			/// Initialization for 4 Order SynGen
			void setOperationalParametersPerUnit(Real nomPower, 
				Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
				Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t);
			/// Initialization for 6 Order SynGen
			void setOperationalParametersPerUnit(Real nomPower, 
				Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
				Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t,
				Real Ld_s, Real Lq_s, Real Td0_s, Real Tq0_s,
				Real Taa=0);
			///
			void setInitialValues(Complex initComplexElectricalPower, 
				Real initMechanicalPower, Complex initTerminalVoltage);

			/// ### Setters ###
			void scaleInertiaConstant(Real scalingFactor); 

			/// ### Mna Section ###
			class MnaPreStep : public Task {
				public:
					MnaPreStep(ReducedOrderSynchronGenerator<VarType>& synGen)
					: Task(synGen.mName + ".MnaPreStep"), mSynGen(synGen) {
				    	mModifiedAttributes.push_back(synGen.attribute("right_vector"));
				    	mPrevStepDependencies.push_back(synGen.attribute("v_intf"));
				}
				void execute(Real time, Int timeStepCount);

				private:
				ReducedOrderSynchronGenerator<VarType>& mSynGen;
			};

			class MnaPostStep : public Task {
			public:
				MnaPostStep(ReducedOrderSynchronGenerator<VarType>& synGen, Attribute<Matrix>::Ptr leftSideVector) :
					Task(synGen.mName + ".MnaPostStep"), 
					mSynGen(synGen), mLeftVector(leftSideVector) {
					mAttributeDependencies.push_back(mLeftVector);
					mModifiedAttributes.push_back(synGen.attribute("v_intf"));
				}
				void execute(Real time, Int timeStepCount);
			private:
				ReducedOrderSynchronGenerator<VarType>& mSynGen;
				Attribute<Matrix>::Ptr mLeftVector;
			};
	};
}
}
