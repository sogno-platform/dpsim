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
#include <cps/EMT/EMT_Ph3_Resistor.h>
#include <cps/EMT/EMT_Ph3_Inductor.h>
#include <cps/EMT/EMT_Ph3_Capacitor.h>
#include <cps/EMT/EMT_Ph3_ControlledVoltageSource.h>
#include <cps/EMT/EMT_Ph3_Transformer.h>
#include <cps/Base/Base_AvVoltageSourceInverterDQWithStateSpace.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	class AvVoltageSourceInverterDQ :
		public Base::AvVoltageSourceInverterDQWithStateSpace,
		public SimPowerComp<Real>,
		public MNAInterface,
		public SharedFactory<AvVoltageSourceInverterDQ> {
	protected:

		// ### Subcomponents ###
		/// Controlled voltage source
		std::shared_ptr<EMT::Ph3::ControlledVoltageSource> mSubCtrledVoltageSource;
		/// Resistor Rf as part of LCL filter
		std::shared_ptr<EMT::Ph3::Resistor> mSubResistorF;
		/// Capacitor Cf as part of LCL filter
		std::shared_ptr<EMT::Ph3::Capacitor> mSubCapacitorF;
		/// Inductor Lf as part of LCL filter
		std::shared_ptr<EMT::Ph3::Inductor> mSubInductorF;
		/// Resistor Rc as part of LCL filter
		std::shared_ptr<EMT::Ph3::Resistor> mSubResistorC;
		/// Optional connection transformer
		std::shared_ptr<EMT::Ph3::Transformer> mConnectionTransformer;

		// ### inputs ###
		///
		Matrix mVcdq = Matrix::Zero(2, 1);
		///
		Matrix mVcabc = Matrix::Zero(3, 1);
		///
		Matrix mIrcdq = Matrix::Zero(2, 1);
		///
		Matrix mIrcabc = Matrix::Zero(3, 1);

		// ### outputs ###
		///
		Matrix mVsdq = Matrix::Zero(2, 1);
		///
		Matrix mVsabc = Matrix::Zero(3, 1);

		/// in case variable time step simulation should be developed in the future
		Real mTimeStep;
		///
		Bool mCtrlOn = true;
		///
		Bool mWithConnectionTransformer=false;
		/// instantaneous omega
		Real mOmegaInst = 0;
		/// instantaneous frequency
		Real mFreqInst = 0;

		///
		std::vector<Real>* mGenProfile = nullptr;
		///
		std::vector<Real>::iterator mCurrentPower;
		///
		Attribute<Real>::Ptr mQRefInput;

		// #### solver ####
		///
		std::vector<const Matrix*> mRightVectorStamps;

	public:
		///
		AvVoltageSourceInverterDQ(String name, Logger::Level logLevel = Logger::Level::off)
			: AvVoltageSourceInverterDQ(name, name, logLevel) {}
		///
		AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false);

		///
		void setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref);
		///
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
			Real ratedPower, Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega);
		///
		void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl,
			Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff);
		///
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		///
		void setInitialStateValues(Real thetaPLLInit, Real phiPLLInit, Real pInit, Real qInit,
			Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);

		///
		SimPowerComp<Real>::Ptr clone(String copySuffix);

		///
		void initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		///
		void updateInputStateSpaceModel(const Matrix& leftVector, Real time);
		///
		void updateStates();
		///
		void updateBMatrixStateSpaceModel();

		///
		Matrix getParkTransformMatrixPowerInvariant(Real theta);
		///
		Matrix getInverseParkTransformMatrixPowerInvariant(Real theta);
		///
		Matrix parkTransformPowerInvariant(Real theta, Real fa, Real fb, Real fc);
		///
		Matrix inverseParkTransformPowerInvariant(Real theta, Real fd, Real fq);

		///
		void step(Real time, Int timeStepCount);
		///
		void updatePowerGeneration();
		// #### General ####
		///
		//void addGenProfile(std::vector<Real>* genProfile);
		///
		void addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber);
		///
		void updateSetPoint(Real time);
		///
		//void initialize(Matrix frequencies);
		///
		void initializeFromPowerflow(Real frequency);
		// #### interface with villas node ####
		void ctrlReceiver(Attribute<Real>::Ptr qref);
		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPreStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
				//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("P_ref"));
				mPrevStepDependencies.push_back(AvVoltageSourceInverterDQ.attribute("i_intf"));
				mPrevStepDependencies.push_back(AvVoltageSourceInverterDQ.attribute("v_intf"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class AddBStep : public Task {
		public:
			AddBStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".AddBStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubInductorF->attribute("right_vector"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mConnectionTransformer->attribute("right_vector"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCapacitorF->attribute("right_vector"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("right_vector"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};


		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ, Attribute<Matrix>::Ptr leftVector) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPostStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ), mLeftVector(leftVector)
			{
				mAttributeDependencies.push_back(leftVector);
				//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("Q_ref"));
				//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mQRefInput);
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("i_intf"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubResistorF->attribute("i_intf"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubInductorF->attribute("i_intf"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mConnectionTransformer->attribute("i_intf"));
				mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubResistorC->attribute("i_intf"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("i_intf"));
				mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
}
}
