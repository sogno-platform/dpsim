/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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
#include <cps/SP/SP_Ph3_Resistor.h>
#include <cps/SP/SP_Ph3_Inductor.h>
#include <cps/SP/SP_Ph3_Capacitor.h>
#include <cps/SP/SP_Ph3_ControlledVoltageSource.h>


namespace CPS {
	namespace SP {
		namespace Ph3 {

			/*

			 an Average Voltage Source Inverter model with
				- controller modeled in state space
				- filter stamped into the global admittance matrix
			*/
			class AvVoltageSourceInverterDQ :
				public PowerComponent<Complex>,
				public MNAInterface,
				public SharedFactory<AvVoltageSourceInverterDQ> {
			protected:
				Complex mVoltNom;

				Real mTimeStep;
				/// Inner voltage source that represents the AvVoltageSourceInverterDQ
				std::shared_ptr<ControlledVoltageSource> mSubCtrledVoltageSource;
				/// get "measurements" from other components
				std::shared_ptr<Resistor> mResistorF;
				std::shared_ptr<Capacitor> mCapacitorF;
				std::shared_ptr<Inductor> mInductorF;

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
				Real mThetaPLL;
				Real mPhiPLL;
				Real mP;
				Real mQ;
				Real mPhi_d;
				Real mPhi_q;
				Real mGamma_d;
				Real mGamma_q;

				/// measurements
				Matrix mIgabc = Matrix::Zero(3, 1);
				Matrix mVcabc = Matrix::Zero(3, 1);
				Matrix mIfabc = Matrix::Zero(3, 1);
				Matrix mVcdq = Matrix::Zero(2, 1);
				Matrix mIgdq = Matrix::Zero(2, 1);
				Matrix mIfdq = Matrix::Zero(2, 1);
				Real mOmegaInst = 0;
				Real mFreqInst = 0;

				// output
				Matrix mVsdq = Matrix::Zero(2, 1);

				// #### Matrices ####
				Matrix mStates;
				// u_old
				Matrix mU;
				/// output
				Matrix mA;
				Matrix mB;
				Matrix mC;
				Matrix mD;

				/// dq to dynamic phasor
				void dqToSP(Real time);

			public:
				AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				AvVoltageSourceInverterDQ(String name,
					Logger::Level logLevel = Logger::Level::off) :AvVoltageSourceInverterDQ(name, name, logLevel) {}
				/// add measurements for Vcabc and Ifabc
				void addMonitoredNodes(std::shared_ptr<Resistor> resistor, std::shared_ptr<Capacitor> cap, std::shared_ptr<Inductor> inductor);

				void updateMonitoredValues(const Matrix& leftVector, Real time);

				void initializeStates(Real omega, Real timeStep,
					Attribute<Matrix>::Ptr leftVector);

				void updateStates();

				void setParameters(Real sysOmega, Complex sysVoltNom, Real Pref, Real Qref, Real Kp_pll, Real Ki_pll,
					Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Lf, Real Cf,
					Real Rf, Real Rc);

				void setControllerParameters(Real Kp_pll, Real Ki_pll,
					Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl);

				// update B,C,D matrices with new theta_pll
				void updateLinearizedModel();

				Matrix getParkTransformMatrix(Real theta);
				Matrix getInverseParkTransformMatrix(Real theta);
				Matrix parkTransform(Real theta, Real fa, Real fb, Real fc);
				Matrix inverseParkTransform(Real theta, Real fd, Real fq, Real zero = 0.);
				Complex syncFrame2to1(Complex f, Real theta1, Real theta2);
				///
				void step(Real time);
				///
				void initializeFromPowerflow(Real frequency);

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
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("P_ref"));
						mAttributeDependencies.push_back(AvVoltageSourceInverterDQ.attribute("Q_ref"));
						mPrevStepDependencies.push_back(mAvVoltageSourceInverterDQ.attribute("i_intf"));
						mPrevStepDependencies.push_back(mAvVoltageSourceInverterDQ.attribute("v_intf"));
						mModifiedAttributes.push_back(mAvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("v_intf"));

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


				class MnaPostStep : public CPS::Task {
				public:
					MnaPostStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ, Attribute<Matrix>::Ptr leftVector) :
						Task(AvVoltageSourceInverterDQ.mName + ".MnaPostStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ), mLeftVector(leftVector)
					{
						mAttributeDependencies.push_back(mLeftVector);
						mAttributeDependencies.push_back(mAvVoltageSourceInverterDQ.mSubCtrledVoltageSource->attribute("i_intf"));
						mAttributeDependencies.push_back(mAvVoltageSourceInverterDQ.mResistorF->attribute("i_intf"));
						mAttributeDependencies.push_back(mAvVoltageSourceInverterDQ.mInductorF->attribute("i_intf"));
						mAttributeDependencies.push_back(mAvVoltageSourceInverterDQ.mCapacitorF->attribute("i_intf"));
						mModifiedAttributes.push_back(mAvVoltageSourceInverterDQ.attribute("i_intf"));
						mModifiedAttributes.push_back(mAvVoltageSourceInverterDQ.attribute("v_intf"));

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
