/** Voltage behind reactance (EMT)
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
namespace EMT {
namespace Ph3 {
	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability

	class SynchronGeneratorVBRStandalone:
		public PowerComponent<Real>,
		public MNAInterface,
		public Base::SynchronGenerator,
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

		/// d dynamic flux
		Real mDPsid;
		/// q dynamic flux
		Real mDPsiq;
		/// Dynamic d voltage
		Real mDVq;
		/// Dynamic q voltage
		Real mDVd;
		/// Dynamic voltage phase a
		Real mDVa;
		/// Dynamic voltage phase b
		Real mDVb;
		/// Dynamic voltage phase c
		Real mDVc;

		/// Interface voltage phase a
		Real mVa;
		/// Interface voltage phase b
		Real mVb;
		/// Interface voltage phase c
		Real mVc;

		/// Interface curent phase a
		Real mIa;
		/// Interface curent phase b
		Real mIb;
		/// Interface curent phase c
		Real mIc;

		/// Interface curent phase a last time step
		Real mIa_hist;
		/// Interface curent phase b last time step
		Real mIb_hist;
		/// Interface curent phase c last time step
		Real mIc_hist;

		///Load Resistance phase a
		Real Ra;
		///Load Resistance phase b
		Real Rb;
		///Load Resistance phase c
		Real Rc;

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
		/// Derivative of inductance matrix
		Matrix pmDInductanceMat = Matrix::Zero(3, 3);

		/// Load Resistance
		Matrix R_load = Matrix::Zero(3, 3);

		/// Phase currents in pu
		Matrix mIabc = Matrix::Zero(3, 1);
		/// Subtransient voltage in pu
		Matrix mDVabc = Matrix::Zero(3, 1);
		Matrix mDVabc_hist = Matrix::Zero(3, 1);

		/// Matrix paremeters for integration of rotor flux linkages - A
		Matrix A_flux = Matrix::Zero(4, 4);
		/// Variables for integration of rotor flux linkages - B
		Matrix B_flux = Matrix::Zero(4, 2);
		/// Variables for integration of rotor flux linkages - C
		Matrix C_flux = Matrix::Zero(4, 1);

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
		void initialize(Real om, Real dt,
			Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod);

		/// Park transform as described in Krause
		Matrix parkTransform(Real theta, Real a, Real b, Real c);

		/// Inverse Park transform as described in Krause
		Matrix inverseParkTransform(Real theta, Real q, Real d, Real zero);

		/// Calculate inductance Matrix L and its derivative
		void CalculateLandpL();

		Matrix& rotorFluxes() { return mRotorFlux; }
		Matrix& dqStatorCurrents() { return mDqStatorCurrents; }
		Real electricalTorque() { return mElecTorque*mBase_T; }
		Real rotationalSpeed() { return mOmMech*mBase_OmMech; }
		Real rotorPosition() { return mThetaMech; }
		/// Performs with the model of a synchronous generator
		/// to calculate the flux and current from the voltage vector.
		void mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time);
		///
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) { }
		/// Retrieves calculated voltage from simulation for next step
		void mnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time);
	};
}
}
}
