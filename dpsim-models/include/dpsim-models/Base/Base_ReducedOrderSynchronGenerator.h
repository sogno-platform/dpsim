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
#include <dpsim-models/Base/Base_Exciter.h>
#include <dpsim-models/Base/Base_PSS.h>
#include <dpsim-models/Base/Base_Governor.h>
#include <dpsim-models/Base/Base_Turbine.h>

namespace CPS {
namespace Base {

	template <typename VarType>
	class ReducedOrderSynchronGenerator :
		public MNASimPowerComp<VarType> {

		public:
			// ### State variables [p.u.]###
			/// dq stator terminal voltage
			/// (0,0) = Vd
			/// (1,0) = Vq
			/// (2,0) = V0
			const Attribute<Matrix>::Ptr mVdq0;
			/// dq0 armature current
			/// (0,0) = Id
			/// (1,0) = Iq
			/// (2,0) = I0
			const Attribute<Matrix>::Ptr mIdq0;
			/// dq stator terminal voltage
			/// (0,0) = Vd
			/// (1,0) = Vq
			const Attribute<Matrix>::Ptr mVdq;
			/// dq armature current
			/// (0,0) = Id
			/// (1,0) = Iq
			const Attribute<Matrix>::Ptr mIdq;
			/// stator electrical torque
			const Attribute<Real>::Ptr mElecTorque;
			/// Mechanical torque
			const Attribute<Real>::Ptr mMechTorque;
			Real mMechTorque_prev;
			/// Rotor speed
			const Attribute<Real>::Ptr mOmMech;
			/// mechanical system angle (between d-axis and stator a-axis)
			const Attribute<Real>::Ptr mThetaMech;
			/// Load angle (between q-axis and stator a-axis)
			const Attribute<Real>::Ptr mDelta;
			/// induced emf by the field current under no-load conditions at time k+1 (p.u.)
			const Attribute<Real>::Ptr mEf;
			/// induced emf by the field current under no-load conditions at time k (p.u.)
			Real mEf_prev;

			/// Destructor - does nothing.
			virtual ~ReducedOrderSynchronGenerator() { }
			/// modelAsCurrentSource=true --> SG is modeled as current source, otherwise as voltage source
			/// Both implementations are equivalent, but the current source implementation is more efficient
			virtual void setModelAsNortonSource(Bool modelAsCurrentSource);
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
			/// Taa=0 for 6b Order SynGen
			void setOperationalParametersPerUnit(Real nomPower,
				Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
				Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t,
				Real Ld_s, Real Lq_s, Real Td0_s, Real Tq0_s,
				Real Taa=0);
			///
			void setInitialValues(Complex initComplexElectricalPower,
				Real initMechanicalPower, Complex initTerminalVoltage);
			///
			void scaleInertiaConstant(Real scalingFactor);

			// ### Controllers ###
			/// Add automatic voltage regulator
			void addExciter(CPS::Base::ExciterParameters exciterParameters, ExciterType exciterType = ExciterType::DC1Simp);
			/// Add automatic voltage regulator
			void addExciter(std::shared_ptr<Base::Exciter> exciter);
			/// Add power system stabilizer
			void addPSS(std::shared_ptr<Base::PSS> PSS);
			/// Add Governor/TurbineGovernor
			void addGovernor(std::shared_ptr<Base::Governor> governor);
			/// Add Governor/TurbineGovernor
			void addTurbine(std::shared_ptr<Base::Turbine> turbine);

		protected:

			using MNASimPowerComp<VarType>::mRightVector;
			using MNASimPowerComp<VarType>::mIntfVoltage;
			using MNASimPowerComp<VarType>::MnaPreStep;
			using MNASimPowerComp<VarType>::MnaPostStep;

			///
			ReducedOrderSynchronGenerator(String uid, String name, Logger::Level logLevel);
			///
			void calculateVBRconstants();
			///
			void calculateResistanceMatrixConstants();
			///
			virtual void initializeResistanceMatrix() = 0;
			///
			void initializeFromNodesAndTerminals(Real frequency);
			/// Function to initialize the specific variables of each SG model
			virtual void specificInitialization() = 0;
			/// Model specific step
        	virtual void stepInPerUnit() = 0;

			// ### MNA Section ###
        	///
        	void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
			/// Add MNA pre step dependencies
			void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
			void mnaCompPreStep(Real time, Int timeStepCount) override;
			/// Add MNA post step dependencies
			void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
			void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) final;
			virtual void mnaCompPostStep(const Matrix& leftVector) = 0;
			/// Stamps system matrix
			virtual void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) = 0;

			// Model flags
			/// Model flag indicating whether the machine is modelled as Norton or Thevenin equivalent
			Bool mModelAsNortonSource;
			// Model flag indicating the SG order to be used
			SGOrder mSGOrder;

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

			// ### VBR constants ###
			///
			Real mAd_t = 0;
			///
			Real mBd_t = 0;
			///
			Real mAq_t = 0;
			///
			Real mBq_t = 0;
			///
			Real mDq_t = 0;
			///
			Real mAd_s = 0;
			///
			Real mAq_s = 0;
			///
			Real mBd_s = 0;
			///
			Real mBq_s = 0;
			///
			Real mCd_s = 0;
			///
			Real mCq_s = 0;
			///
			Real mDq_s = 0;
			///
			Real mYd = 0;
			///
			Real mYq = 0;

			// ### Constants of resistance matrix (VBR) ###
			///
			Real mA = 0;
			///
			Real mB = 0;

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

			/// Flag to remember when initial values are set
			Bool mInitialValuesSet = false;

			// #### Controllers ####
			/// Determines if Exciter is activated
			Bool mHasExciter = false;
			/// Determines if Exciter is activated
			Bool mHasPSS = false;
			//Determines if generator has a turbine
			Bool mHasTurbine = false;
			//Determines if turbine has a Governor/TurbineGovernor
			Bool mHasGovernor = false;
			
			/// Signal component modelling voltage regulator and exciter
			std::shared_ptr<Base::Exciter> mExciter;
			/// Signal component modelling power system stabilizer
			std::shared_ptr<Base::PSS> mPSS;
			/// Signal component modelling governor control
			std::shared_ptr<Base::Governor> mGovernor;
			/// Signal component modelling Turbine
			std::shared_ptr<Base::Turbine> mTurbine;
			
			/// 
			Real mVpss = 0;

			///
			Real mTimeStep;
			///
			Real mSimTime;
	};
}
}
