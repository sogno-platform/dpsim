/** Voltage behind reactance (DP)
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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
#include <cps/Base/Base_SynchronGenerator.h>
#include <cps/Signal/Exciter.h>
#include <cps/Signal/TurbineGovernor.h>

namespace CPS {
namespace DP {
namespace Ph3 {
	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability
	class SynchronGeneratorVBRStandalone:
		public PowerComponent<Complex>,
		public Base::SynchronGenerator,
		public MNAInterface,
		public SharedFactory<SynchronGeneratorVBRStandalone> {

	protected:
		/// Exciter Model
		//Signal::Exciter mExciter;
		/// Determine if Exciter is activated
		bool mHasExciter = false;

		/// Governor Model
		//Signal::TurbineGovernor mTurbineGovernor;
		/// Determine if Turbine and Governor are activated
		bool mHasTurbineGovernor = false;

		/// d dynamic inductance
		Real mDLmd;
		/// q dynamic inductance
		Real mDLmq;

		/// Auxiliar inductance
		Real mLa;
		/// Auxiliar inductance
		Real mLb;

		//Real mThetaMech;
		Real mThetaMech2;
		/// initial theta
		Real mTheta0;

		/// d dynamic flux
		Real mDPsid;
		/// q dynamic flux
		Real mDPsiq;
		/// Dynamic d voltage
		Real mDVq;
		/// Dynamic q voltage
		Real mDVd;
		/// Dynamic voltage phase a _ Real part
		Real mDVaRe;
		/// Dynamic voltage phase a _ Imaginary part
		Real mDVaIm;
		/// Dynamic voltage phase b _ Real part
		Real mDVbRe;
		/// Dynamic voltage phase b _ Imaginary part
		Real mDVbIm;
		/// Dynamic voltage phase c _ Real part
		Real mDVcRe;
		/// Dynamic voltage phase c _ Imaginary part
		Real mDVcIm;

		/// Interface voltage phase a _ Real part
		Real mVaRe;
		/// Interface voltage phase a _ Imaginary part
		Real mVaIm;
		/// Interface voltage phase b _ Real part
		Real mVbRe;
		/// Interface voltage phase b _ Imaginary part
		Real mVbIm;
		/// Interface voltage phase c _ Real part
		Real mVcRe;
		/// Interface voltage phase c _ Imaginary part
		Real mVcIm;

		/// Interface current phase a _ Real part
		Real mIaRe;
		/// Interface current phase a _ Imaginary part
		Real mIaIm;
		/// Interface current phase b _ Real part
		Real mIbRe;
		/// Interface current phase b _ Imaginary part
		Real mIbIm;
		/// Interface current phase c _ Real part
		Real mIcRe;
		/// Interface current phase c _ Imaginary part
		Real mIcIm;

		/// Magnetizing flux linkage in q axis
		Real mPsimq;
		/// Magnetizing flux linkage in d axis
		Real mPsimd;

		/// Rotor flux vector
		Matrix mRotorFlux = Matrix::Zero(4, 1);
		/// Dq stator current vector
		Matrix mDqStatorCurrents = Matrix::Zero(2, 1);
		/// Dq stator current vector - from previous time step
		Matrix mDqStatorCurrents_hist = Matrix::Zero(2, 1);

		// ### Useful Matrices ###
		/// inductance matrix
		Matrix mDInductanceMat = Matrix::Zero(3, 3);
		/// load resistance matrix
		Matrix R_load = Matrix::Zero(6, 6);
		/// Constant part of equivalent stator inductance
		Matrix LD0 = Matrix::Zero(3, 3);
		/// Equivalent stator inductance matrix
		Matrix L_EQ = Matrix::Zero(6, 6);
		/// Equivalent stator resistance matrix
		Matrix R_EQ = Matrix::Zero(6, 6);

		/// Interfase phase current vector
		Matrix mIabc = Matrix::Zero(6, 1);
		/// Dynamic Voltage vector
		Matrix mDVabc = Matrix::Zero(6, 1);
		/// Dynamic Voltage vector
		Matrix mDVabc_hist = Matrix::Zero(6, 1);

		/// Matrix paremeters for integration of rotor flux linkages - A
		Matrix A_flux = Matrix::Zero(4, 4);
		/// Variables for integration of rotor flux linkages - B
		Matrix B_flux = Matrix::Zero(4, 2);
		/// Variables for integration of rotor flux linkages - C
		Matrix C_flux = Matrix::Zero(4, 1);

		/// Q axis Rotor flux
		Matrix mPsikq1kq2 = Matrix::Zero(2, 1);
		/// D axis rotor flux
		Matrix mPsifdkd = Matrix::Zero(2, 1);

		/// Park Transformation Matrix
		Matrix mKrs_teta = Matrix::Zero(3, 3);
		/// Inverse Park Transformation Matrix
		Matrix mKrs_teta_inv = Matrix::Zero(3, 3);

		/// Auxiliar variables
		Real c21_omega;
		Real c22_omega;
		Real c13_omega;
		Real c14_omega;
		Matrix K1a = Matrix::Zero(2, 2);
		Matrix K1b = Matrix::Zero(2, 1);
		Matrix K1 = Matrix::Zero(2, 1);
		Matrix K2a = Matrix::Zero(2, 2);
		Matrix K2b = Matrix::Zero(2, 1);
		Matrix K2 = Matrix::Zero(2, 1);
		Matrix h_qdr;
		Matrix K = Matrix::Zero(2, 2);
		Matrix E_r_vbr = Matrix::Zero(3, 1);
		Matrix K_DP = Matrix::Zero(6, 6);

		/// Auxiliar constants
		Real c11;
		Real c12;
		Real c23;
		Real c24;
		Real c15;
		Real c25;
		Real c26;
		Real b11;
		Real b12;
		Real b13;
		Real b21;
		Real b22;
		Real b23;
		Real b31;
		Real b32;
		Real b33;
		Real b41;
		Real b42;
		Real b43;


		Real E1_1d;
		Real E2_1d;

		Matrix Ea = Matrix::Zero(2, 2);
		Matrix E1b = Matrix::Zero(2, 1);
		Matrix E1 = Matrix::Zero(2, 2);
		Matrix Fa = Matrix::Zero(2, 2);
		Matrix F1b = Matrix::Zero(2, 1);
		Matrix F1 = Matrix::Zero(2, 2);
		Matrix E2b = Matrix::Zero(2, 2);
		Matrix E2 = Matrix::Zero(2, 2);
		Matrix F2b = Matrix::Zero(2, 2);
		Matrix F2 = Matrix::Zero(2, 2);
		Matrix F3b = Matrix::Zero(2, 1);
		Matrix F3 = Matrix::Zero(2, 2);
		Matrix C26 = Matrix::Zero(2, 1);
		//Matrix A = Matrix::Zero(6, 6);
		//Matrix B = Matrix::Zero(6, 6);
		//Matrix Var1 = Matrix::Zero(6, 6);
		//Matrix Var2 = Matrix::Zero(6, 6);
		//Matrix mVabc = Matrix::Zero(6, 1);
		Matrix E_r_vbr_DP = Matrix::Zero(6, 1);


	public:
		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		SynchronGeneratorVBRStandalone(String name,
			Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia, Logger::Level logLevel = Logger::Level::off);

		/// Function to initialize Exciter
		void addExciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd);
		/// Function to initialize Governor and Turbine
		void addGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
		/// Function parameters have to be given in real units.
		void initialize(Real omega, Real timeStep,
			Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector.
		void mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod);

		/// Retrieves calculated voltage from simulation for next step
		void mnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time) { }

		/// abc to dq
		Matrix abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm);

		/// dq to abc
		Matrix dq0ToAbcTransform(Real theta, Real d, Real q, Real zero);

		void CalculateLandR(Real time);
		void CalculateAuxiliarConstants(Real dt);
		void CalculateAuxiliarVariables(Real time);

		Matrix& rotorFluxes() { return mRotorFlux; }
		Matrix& dqStatorCurrents() { return mDqStatorCurrents; }
		Real electricalTorque() { return mElecTorque*mBase_T; }
		Real rotationalSpeed() { return mOmMech*mBase_OmMech; }
		Real rotorPosition() { return mThetaMech; }
		Matrix& statorCurrents() { return mIabc; }

	};
}
}
}
