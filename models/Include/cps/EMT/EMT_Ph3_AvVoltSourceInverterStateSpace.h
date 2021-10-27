/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Base/Base_Ph1_VoltageSource.h>
#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

	/// average inverter model with LC filter
	class AvVoltSourceInverterStateSpace :
		public MNAInterface,
		public SimPowerComp<Real>,
		public Base::Ph1::VoltageSource,
		public SharedFactory<AvVoltSourceInverterStateSpace> {
	protected:
		Real mTimeStep;
		// ### parameters ###
		Real mPref;
		Real mQref;

		/// filter paramter
		Real mLf;
		Real mCf;
		Real mRf;

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

		/// connection to grid
		Real mRc;

		// states

		Real mThetaPLL;
		Real mPhiPLL;

		Real mP;
		Real mQ;

		Real mPhi_d;
		Real mPhi_q;

		Real mGamma_d;
		Real mGamma_q;

		Matrix mIfabc = Matrix::Zero(3, 1);
		/*Real mIfa;
		Real mIfb;
		Real mIfc;*/

		Matrix mVcabc = Matrix::Zero(3, 1);
		/*Real mVca;
		Real mVcb;
		Real mVcc;*/

		// Norton equivalant voltage source
		Matrix mEquivCurrent = Matrix::Zero(3, 1);
		//  ### Real Voltage source parameters ###
		/// conductance of mRc[S]
		Real mYc;
		// #### Matrices ####
		Matrix mStates;
		// u_old
		Matrix mU;
		/// output
		Matrix mIg_abc = Matrix::Zero(3, 1);
		Matrix mA;
		Matrix mB;
		Matrix mC;
		Matrix mD;
		// park transform matrix
		Matrix mParkTransform;

	public:
		AvVoltSourceInverterStateSpace(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		AvVoltSourceInverterStateSpace(String name,
			Logger::Level logLevel = Logger::Level::off) :AvVoltSourceInverterStateSpace(name, name, logLevel) {}

		// initialize with parameters already set.
		// sysVoltNom: phase voltage

		void initializeStates(Real omega, Real timeStep,
			Attribute<Matrix>::Ptr leftVector);

		// initialize with parameters.
		//void initialize(Real theta, Real phi_pll, Real p, Real q, Real phi_d, Real phi_q,
		//	Real gamma_d, Real gamma_q, Real i_fd, Real i_fq, Real v_cd, Real v_cq);

		//void initStates(Real initOmegaPLL, Real initPhiPLL, Real initP, Real initQ,
		//	Real initPhid, Real initPhiQ, Real initGamma_d, Real initGamma_q, Real initVcabc, Real initIfabc);

		void updateStates();

		void setParameters(Real sysOmega, Complex sysVoltNom, Real Pref, Real Qref, Real Lf, Real Cf, Real Rf, Real Rc, Real Kp_pll, Real Ki_pll,
			Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl);

		void setFilterParameters(Real Lf, Real Cf, Real Rf);

		void setControllerParameters(Real Kp_pll, Real Ki_pll,
			Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl);

		//update park transform coefficients inside A, B matrices according to the new states (thea_pll)
		//update Ig_abc in matrix B
		void updateLinearizedCoeffs();

		Matrix getParkTransformMatrix(Real theta);
		Matrix getInverseParkTransformMatrix(Real theta);
		Matrix parkTransform(Real theta, Real fa, Real fb, Real fc);
		Matrix inverseParkTransform(Real theta, Real fd, Real fq, Real zero = 0.);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// update equivalent current of the equivalent source
		void updateEquivCurrent(Real time);

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(AvVoltSourceInverterStateSpace& avVoltSourceInverterStateSpace) :
				Task(avVoltSourceInverterStateSpace.mName + ".MnaPreStep"), mAvVoltSourceInverterStateSpace(avVoltSourceInverterStateSpace) {
				mAttributeDependencies.push_back(avVoltSourceInverterStateSpace.attribute("P_ref"));
				mModifiedAttributes.push_back(mAvVoltSourceInverterStateSpace.attribute("right_vector"));
				mModifiedAttributes.push_back(mAvVoltSourceInverterStateSpace.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltSourceInverterStateSpace& mAvVoltSourceInverterStateSpace;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(AvVoltSourceInverterStateSpace& avVoltSourceInverterStateSpace, Attribute<Matrix>::Ptr leftVector) :
				Task(avVoltSourceInverterStateSpace.mName + ".MnaPostStep"), mAvVoltSourceInverterStateSpace(avVoltSourceInverterStateSpace), mLeftVector(leftVector)
			{
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mAvVoltSourceInverterStateSpace.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			AvVoltSourceInverterStateSpace& mAvVoltSourceInverterStateSpace;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
}
}
