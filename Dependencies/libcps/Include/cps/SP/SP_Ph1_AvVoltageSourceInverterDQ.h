/**
 * @file
 * @Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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
#include <cps/Definitions.h>
#include <cps/SP/SP_Ph1_Resistor.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Capacitor.h>
#include <cps/SP/SP_Ph1_ControlledVoltageSource.h>
#include <cps/SP/SP_Ph1_Load.h>


namespace CPS {
	namespace SP {
		namespace Ph1 {
			/*
				 Average voltage source inverter
				 - modelled in dq
				 - with interface to SP grid
			*/
			class AvVoltageSourceInverterDQ :
				public PowerComponent<Complex>,
				public MNAInterface,
				public PFSolverInterfaceBus,
				public SharedFactory<AvVoltageSourceInverterDQ> {
			protected:
				Complex mVoltNom;

				/// in case variable time step simulation should be developed in the future
				Real mTimeStep;
				/// if SteadyStateInit is enabled, the system's sychronous frame will start from a certain angle 
				Real mThetaSInit = 0;
				/// Inner voltage source that represents the AvVoltageSourceInverterDQ
				std::shared_ptr<SP::Ph1::ControlledVoltageSource> mSubCtrledVoltageSource;
				/// LC filter as sub-components
				std::shared_ptr<SP::Ph1::Resistor> mSubResistorF;
				///
				std::shared_ptr<SP::Ph1::Capacitor> mSubCapacitorF;
				///
				std::shared_ptr<SP::Ph1::Inductor> mSubInductorF;
				///
				std::shared_ptr<SP::Ph1::Resistor> mSubResistorC;
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
				///
				Bool mCtrlOn = true;
				///
				Bool mCoveeCtrled=true;
				///
				Bool mIsLoad=false;

				// ### parameters ###
				Real mPref;
				Real mQref;

				/// filter paramter
				Real mLf;
				Real mCf;
				Real mRf;

				Real mRc;

				/// PLL
				Real mOmegaN;
				Real mKiPLL;
				Real mKpPLL;

				/// Power controller
				Real mOmegaCutoff;
				Real mKiPowerCtrld;
				Real mKiPowerCtrlq;
				Real mKpPowerCtrld;
				Real mKpPowerCtrlq;

				/// Current controller
				Real mKiCurrCtrld;
				Real mKiCurrCtrlq;
				Real mKpCurrCtrld;
				Real mKpCurrCtrlq;

				/// states
				Real mThetaPLL = 0;
				Real mPhiPLL;
				Real mP;
				Real mQ;
				Real mPhi_d;
				Real mPhi_q;
				Real mGamma_d;
				Real mGamma_q;

				/// measurements
				Matrix mVcdq = Matrix::Zero(2, 1);
				Matrix mIgdq = Matrix::Zero(2, 1);
				Matrix mIfdq = Matrix::Zero(2, 1);
				/// instantaneous omega
				Real mOmegaInst = 0;
				/// instantaneous frequency
				Real mFreqInst = 0;

				/// output
				Matrix mVsdq = Matrix::Zero(2, 1);

				// #### Matrices ####
				Matrix mStates;
				/// u_old
				Matrix mU;
				/// output
				Matrix mA;
				Matrix mB;
				Matrix mC;
				Matrix mD;
				/// dq to dynamic phasor
				MatrixComp dqToSP(Real time);

				// #### solver ####
				///
				std::vector<const Matrix*> mRightVectorStamps;

			public:
				///
				std::shared_ptr<SP::Ph1::Load> mPFAvVoltageSourceInverter;
				///
				std::vector<PQData> mLoadProfile;			
				// #### constructors ####
				///
				AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				AvVoltageSourceInverterDQ(String name,
					Logger::Level logLevel = Logger::Level::off) :AvVoltageSourceInverterDQ(name, name, logLevel) {}
				///
				PowerComponent<Complex>::Ptr clone(String copySuffix);
				/// add measurements for Vcabc and Ifabc
				//void addMonitoredNodes( std::shared_ptr<Capacitor> cap);
				///
				void updateMonitoredValues(const Matrix& leftVector, Real time);
				///
				void initializeModel(Real omega, Real timeStep,
					Attribute<Matrix>::Ptr leftVector);
				///
				void updateStates();
				///
				void setParameters(Real sysOmega, Complex sysVoltNom, Real Pref, Real Qref, Real Kp_pll, Real Ki_pll,
					Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Lf, Real Cf,
					Real Rf, Real Rc);
				///
				void setControllerParameters(Real Kp_pll, Real Ki_pll,
					Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl);

				/// update B,C,D matrices with new theta_pll
				void updateLinearizedModel();
				///
				Matrix getParkTransformMatrix(Real theta);
				///
				Matrix getInverseParkTransformMatrix(Real theta);
				///
				Matrix parkTransform(Real theta, Real fa, Real fb, Real fc);
				///
				Matrix inverseParkTransform(Real theta, Real fd, Real fq, Real zero = 0.);
				/// convert between two rotating frames
				Complex rotatingFrame2to1(Complex f, Real theta1, Real theta2);
				///
				void step(Real time, Int timeStepCount);
				///
				void updatePowerGeneration();

				// #### General ####
				///
				void addGenProfile(std::vector<Real>* genProfile);
				///
				void addAggregatedGenProfile(std::vector<Real>* genProfile, Real customerNumber);
				///
				void updateSetPoint(Real time);

				//void initialize(Matrix frequencies);
				///
				void initializeFromPowerflow(Real frequency);

				// #### Powerflow section ####
				 /// Modify powerflow bus type
				void ctrlReceiver(Attribute<Real>::Ptr qrefInput);
				/// update reference set-points
				void updatePQ(Real time);
				///
				void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
				///
				void pfBusInitialize() override;

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				/// Stamps right side (source) vector
				void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				/// Returns current through the component
				void mnaUpdateCurrent(const Matrix& leftVector);
				/// interface with power control service
				void coveeCtrled(Bool ctrled){mCoveeCtrled=ctrled;};
				///
				void makeLoad(Bool isLoad){mIsLoad=isLoad;};
				///
				Bool isLoad(){return mIsLoad;};
				///
				void setProfileUpdateRate(UInt rate){mProfileUndateRate=rate;};
				class MnaPreStep : public CPS::Task {
				public:
					MnaPreStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
						Task(AvVoltageSourceInverterDQ.mName + ".MnaPreStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
						//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("P_ref"));
						//mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("Q_ref"));

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
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("right_vector"));
						mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("right_vector"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
				};

				class CtrlStep : public Task {
				public:
					CtrlStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
						Task(AvVoltageSourceInverterDQ.mName + ".CtrlStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mQRefInput);
						mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("Q_ref"));
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
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("i_intf"));
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubResistorF->attribute("i_intf"));
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubInductorF->attribute("i_intf"));
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mSubResistorC->attribute("i_intf"));
						mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("i_intf"));
						mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("v_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
					Attribute<Matrix>::Ptr mLeftVector;
				};
				
				class PowerFlowStep: public CPS::Task {
					public:
					PowerFlowStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
						Task(AvVoltageSourceInverterDQ.mName + ".PowerFlowStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.mQRefInput);
						mModifiedAttributes.push_back(AvVoltageSourceInverterDQ.attribute("Q_ref"));
					}
					void execute(Real time, Int timeStepCount);

				private:
					AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
				};


			};
		}
	}
}
