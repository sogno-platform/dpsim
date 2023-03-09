/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
	/// Interface to be used by synchronous generators
	class MNASyncGenInterface {
	protected:
		// #### Model specific variables ####
		/// generator terminal at time k-1
		Matrix mVdq_prev;
		/// predicted generator current at time k
		Matrix mIdq_pred;
		/// corrected generator current at time k
		Matrix mIdq_corr;
		/// voltage behind the transient/subtransiet impedance at time k-1
		Attribute<Matrix>::Ptr mEdq;
		/// predicted voltage behind the transient and subtransient impedance at time k+1
		Matrix mEdq_pred;
		/// corrected voltage behind the transient and subtransient impedance at time k+1
		Matrix mEdq_corr;
		/// corrected electrical torque
		Real mElecTorque_corr;
		/// predicted mechanical omega at time k
		Real mOmMech_pred;
		/// corrected mechanical omega at time k
		Real mOmMech_corr;
		/// prediction mechanical system angle at time k
		Real mThetaMech_pred;
		/// corrected mechanical system angle at time k
		Real mThetaMech_corr;
		/// predicted delta at time k
		Real mDelta_pred;
		/// corrected delta at time k
		Real mDelta_corr;

		/// Matrix used when numerical method of predictor step = Euler
		/// State Matrix backward euler: Edq(k) = mA_euler * Edq(k) + mB_euler * Idq + mC_euler * Ef
		Matrix mA_euler;
		Matrix mB_euler;
		Matrix mC_euler;

		/// Matrixes used when numerical method of corrector step = trapezoidal rule
		/// State Matrix trapezoidal rule (corrector step): x(k+1) = mA_prev * Edq(k-1) + mA_corr * Edq_corr(k) + B_corr * Idq_corr(k) + mC_corr * Ef
		Matrix mA_prev;
		Matrix mA_corr;
		Matrix mB_corr;
		Matrix mC_corr;

		/// Transformation matrix dp->dq
		MatrixComp mDpToDq;

		/// Vector to create abc vector from a component
		MatrixComp mShiftVector;

		///
		Attribute<Int>::Ptr mNumIter;
		///
		Int mMaxIter = 25;
		///
		Real mTolerance = 1e-6;

	public:
		typedef std::shared_ptr<MNASyncGenInterface> Ptr;
		typedef std::vector<Ptr> List;

		// Solver functions
		///
		virtual void correctorStep()=0;
		///
		virtual void updateVoltage(const Matrix& leftVector)=0;
		///
		virtual bool requiresIteration() {return false;}

		/// Setters
		///
		void setMaxIterations(Int maxIterations) {mMaxIter = maxIterations;}
		///
		void setTolerance(Real Tolerance) {mTolerance = Tolerance;}

	protected:
		/// Constructor
		MNASyncGenInterface() {
			// inizialize matrix that are not model dependent
			mIdq_pred = Matrix::Zero(2,1);
			mIdq_corr = Matrix::Zero(2,1);

			// Vector to convert 1phase to 3phase

			mShiftVector = Matrix::Zero(3,1);
			mShiftVector << Complex(1., 0), SHIFT_TO_PHASE_B, SHIFT_TO_PHASE_C;
		}

		///
		Matrix parkTransform(Real theta, const Matrix& abcVector) {
			Matrix dq0Vector(3, 1);
			Matrix dqVector(2, 1);
			Matrix abcToDq0(3, 3);

			// Park transform according to Kundur
			abcToDq0 <<
		 		2./3.*cos(theta),	2./3.*cos(theta - 2.*PI/3.),  2./3.*cos(theta + 2.*PI/3.),
				-2./3.*sin(theta), -2./3.*sin(theta - 2.*PI/3.), -2./3.*sin(theta + 2.*PI/3.),
		 		1./3., 			1./3., 						  1./3.;

			dq0Vector = abcToDq0 * abcVector;
			dqVector << dq0Vector(0,0), dq0Vector(1,0);

			return dqVector;
		}

	};
}
