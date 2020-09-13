/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once


#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Definitions.h>
#include <cps/DP/DP_Ph1_Resistor.h>
#include <cps/DP/DP_Ph1_Inductor.h>
#include <cps/DP/DP_Ph1_Capacitor.h>
#include <cps/DP/DP_Ph1_ControlledVoltageSource.h>
#include <cps/DP/DP_Ph1_Transformer.h>
#include <cps/Base/Base_AvVoltageSourceInverterDQ.h>
#include <cps/PowerProfile.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	class AvVoltageSourceInverterDQ :
		public Base::AvVoltageSourceInverterDQ,
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<AvVoltageSourceInverterDQ> {
	protected:
		// ### Electrical Subcomponents ###
		/// Controlled voltage source
		std::shared_ptr<DP::Ph1::ControlledVoltageSource> mSubCtrledVoltageSource;
		/// Resistor Rf as part of LCL filter
		std::shared_ptr<DP::Ph1::Resistor> mSubResistorF;
		/// Capacitor Cf as part of LCL filter
		std::shared_ptr<DP::Ph1::Capacitor> mSubCapacitorF;
		/// Inductor Lf as part of LCL filter
		std::shared_ptr<DP::Ph1::Inductor> mSubInductorF;
		/// Resistor Rc as part of LCL filter
		std::shared_ptr<DP::Ph1::Resistor> mSubResistorC;
		/// Optional connection transformer
		std::shared_ptr<DP::Ph1::Transformer> mConnectionTransformer;

		// ### Inverter Variables ###
		// Control inputs
		///
		Matrix mVcdq = Matrix::Zero(2, 1);
		///
		Matrix mIrcdq = Matrix::Zero(2, 1);
		// Control outputs
		/// Control output before transformation interface
		Matrix mVsdq = Matrix::Zero(2, 1);
		/// Control output before transformation interface
		MatrixComp mControllerOutput = MatrixComp::Zero(1,1);

		/// instantaneous omega
		Real mOmegaInst=0;
		/// instantaneous frequency
		Real mFreqInst=0;
		///
		Bool mIsLoad=false;
		///
		Bool mWithConnectionTransformer=false;
		/// in case variable time step simulation should be developed in the future
		Real mTimeStep;
		///
		std::vector<Real>* mGenProfile = nullptr;
		///
		std::vector<Real>::iterator mCurrentPower;
		///
		std::vector<PQData>::iterator mCurrentLoad;
		///
		UInt mProfileUndateRate = 1000;
		///
		Attribute<Real>::Ptr mQRefInput;

		// #### solver ####
		///
		std::vector<const Matrix*> mRightVectorStamps;

	public:
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff);

		///
		std::vector<PQData> mLoadProfile;

		/// Defines name amd logging level
		AvVoltageSourceInverterDQ(String name, Logger::Level logLevel = Logger::Level::off)
			: AvVoltageSourceInverterDQ(name, name, logLevel) {}
		/// Defines UID, name, logging level and connection trafo existence
		AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		///
		void initialize(Matrix frequencies);
		///
		SimPowerComp<Complex>::Ptr clone(String copySuffix);
		///
		void setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref);
		///
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
			Real ratedPower, Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega);
		///
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		///
		void setInitialStateValues(Real thetaPLLInit, Real phiPLLInit, Real pInit, Real qInit,
			Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Updates current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates voltage across component
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		// #### Control section ####
		///
		void initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		///
		void updateInputStateSpaceModel(Real time);
		///
		void updateStates();
		///
		void updateBMatrixStateSpaceModel();
		/// convert between two rotating frames
		Complex rotatingFrame2to1(Complex f, Real theta1, Real theta2);
		///
		void step(Real time, Int timeStepCount);
		/// Perform step of controller
		void controlStep(Real time, Int timeStepCount);
		/// Add control step dependencies
		void addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		// #### Profile handling ####
		///
		void addGenProfile(std::vector<Real>* genProfile);
		///
		void addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber);
		///
		void makeLoad(Bool isLoad){mIsLoad=isLoad;};
		///
		Bool isLoad(){return mIsLoad;};
		///
		void setProfileUpdateRate(UInt rate){mProfileUndateRate=rate;};
		///
		void updatePowerGeneration();

		class ControlStep : public CPS::Task {
		public:
			ControlStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".ControlStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
					mAvVoltageSourceInverterDQ.addControlStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mAvVoltageSourceInverterDQ.controlStep(time, timeStepCount); };

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPreStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
					mAvVoltageSourceInverterDQ.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mAvVoltageSourceInverterDQ.mnaPreStep(time, timeStepCount); };

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ, Attribute<Matrix>::Ptr leftVector) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPostStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ), mLeftVector(leftVector) {
				mAvVoltageSourceInverterDQ.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mAvVoltageSourceInverterDQ.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
}
}
