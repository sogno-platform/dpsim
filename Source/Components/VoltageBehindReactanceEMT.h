/** Voltage behind reactance (EMT)
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
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

#include "BaseComponent.h"
#include "ComponentCommons.h"

namespace DPsim {

	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability

	class VoltageBehindReactanceEMT : public BaseComponent {
	protected:

		// ### Machine parameters ###
		/// nominal power Pn [VA]
		double mNomPower;
		/// nominal voltage Vn [V] (RMS)
		double mNomVolt;
		/// nominal frequency fn [Hz]
		double mNomFreq;
		/// nominal field current Ifn [A]
		double mNomFieldCur;

		/// stator resistance Rs[Ohm]
		double mRs;
		/// leakage inductance Ll [H]
		double mLl;
		/// d-axis mutual inductance Lmd [H]
		double mLmd;
		/// unsaturated d-axis mutual inductance Lmd [H]
		double mLmd0;
		/// q-axis mutual inductance Lmq [H]
		double mLmq;
		/// unsaturated q-axis mutual inductance Lmq [H]
		double mLmq0;
		/// field resistance Rfd [Ohm]
		double mRfd;
		/// field leakage inductance Llfd [H]
		double mLlfd;
		/// d-axis damper resistance Rkd [Ohm]
		double mRkd;
		/// d-axis damper leakage inductance Llkd [H]
		double mLlkd;
		/// q-axis damper resistance 1 Rkq1 [Ohm]
		double mRkq1;
		/// q-axis damper leakage inductance 1 Llkq1 [H]
		double mLlkq1;
		/// q-axis damper resistance 2 Rkq2 [Ohm]
		double mRkq2;
		/// q-axis damper leakage inductance 2 Llkq2 [H]
		double mLlkq2;

		///Number of damping windings in q
		double DampingWinding = 2;

		/// inertia J [kg*m^2]
		double mJ;
		/// number of poles
		int mPoleNumber;
		/// inertia coefficient H
		double mH;

		/// d dynamic inductance
		double mDLmd;
		/// q dynamic inductance
		double mDLmq;
		/// Auxiliar inductance term La
		double mLa;
		/// Auxiliar inductance term Lb
		double mLb;


		// ### Stator base values ###
		/// specifies if the machine parameters are transformed to per unit
		SynchGenStateType mStateType;
		/// base stator voltage
		double mBase_v;
		/// base stator voltage RMS
		double mBase_V_RMS;
		/// base stator current
		double mBase_i;
		/// base stator current RMS
		double mBase_I_RMS;
		/// base stator impedance
		double mBase_Z;
		/// base electrical angular frequency
		double mBase_OmElec;
		/// base mechanical angular frequency
		double mBase_OmMech;
		/// base stator inductance
		double mBase_L;
		/// base torque
		double mBase_T;
		/// base flux linkage
		double mBase_Psi;

		/// base field current
		double mBase_ifd;
		/// base field voltage
		double mBase_vfd;
		/// base field impedance
		double mBase_Zfd;
		/// base field inductance
		double mBase_Lfd;


		// ### State variables ###
		/// rotor speed omega_r
		double mOmMech;
		/// theta
		double mThetaMech;
		/// mechanical Power Pm [W]
		double mMechPower;
		/// mechanical torque
		double mMechTorque;
		/// electrical torque
		double mElecTorque;

		/// d dynamic flux
		double mDPsid;
		/// q dynamic flux
		double mDPsiq;
		/// Dynamic d voltage
		double mDVq;
		/// Dynamic q voltage
		double mDVd;
		/// Dynamic voltage phase a
		double mDVa;
		/// Dynamic voltage phase b
		double mDVb;
		/// Dynamic voltage phase c
		double mDVc;


		/// stator voltage in d axis
		double mVd;
		/// stator voltage in q axis
		double mVq;
		/// stator voltage 0 component
		double mV0;
		/// Rotor voltage field winding
		double mVfd;
		/// Rotor voltage damping winding in d axis
		double mVkd;
		/// Rotor voltage damping winding 1 in q axis
		double mVkq1;
		/// Rotor voltage damping winding 2 in q axis
		double mVkq2;

		/// stator current in d axis
		double mId;
		/// stator current in q axis
		double mIq;
		/// stator current 0 component
		double mI0;
		/// Rotor current field winding
		double mIfd;
		/// Rotor current damping winding in d axis
		double mIkd;
		/// Rotor current damping winding 1 in q axis
		double mIkq1;
		/// Rotor current damping winding 2 in q axis
		double mIkq2;

		/// stator flux linkage in d axis
		double mPsid;
		/// stator flux linkage in q axis
		double mPsiq;
		/// stator flux linkage 0 component
		double mPsi0;
		/// rotor flux linkage in field winding
		double mPsifd;
		/// rotor flux linkage in damping winding from d axis
		double mPsikd;
		/// rotor flux linkage in damping winding 1 from q axis
		double mPsikq1;
		/// rotor flux linkage in damping winding 2 from q axis
		double mPsikq2;

		/// Interface voltage phase a
		double mVa;
		/// Interface voltage phase b
		double mVb;
		/// Interface voltage phase c
		double mVc;

		/// Interface curent phase a
		double mIa;
		/// Interface curent phase b
		double mIb;
		/// Interface curent phase c
		double mIc;

		/// Magnetizing flux linkage in q axis
		Real mPsimq;
		/// Magnetizing flux linkage in d axis
		Real mPsimd;

		Matrix mRotorFlux = Matrix::Zero(4, 1);
		Matrix mDqStatorCurrents = Matrix::Zero(2, 1);
		Matrix mDqStatorCurrents_hist = Matrix::Zero(2, 1);

		// ### Useful Matrices ###
		/// inductance matrix
		Matrix mDInductanceMat = Matrix::Zero(3, 3);
		/// Derivative of inductance matrix
		Matrix pmDInductanceMat = Matrix::Zero(3, 3);
		/// resistance matrix
		Matrix mResistanceMat = Matrix::Zero(3, 3);

		/// Load Resistance 
		Matrix R_load = Matrix::Zero(3, 3);

		/// Phase Voltages in pu
		Matrix mVabc = Matrix::Zero(3, 1);
		/// Phase currents in pu
		Matrix mIabc = Matrix::Zero(3, 1);
		/// Subtransient voltage in pu
		Matrix mDVabc = Matrix::Zero(3, 1);
		/// Phase Voltages [V]
		Matrix mVoltageVector = Matrix::Zero(3, 1);
		/// Phase Currents [A]
		Matrix mCurrentVector = Matrix::Zero(3, 1);

		/// Matrix paremeters for integration of rotor flux linkages - A
		Matrix A_flux = Matrix::Zero(4, 4);
		/// Variables for integration of rotor flux linkages - B
		Matrix B_flux = Matrix::Zero(4, 2);
		/// Variables for integration of rotor flux linkages - C
		Matrix C_flux = Matrix::Zero(4, 1);


	public:
		VoltageBehindReactanceEMT() { };

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		VoltageBehindReactanceEMT(std::string name, Int node1, Int node2, Int node3,
			Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia);

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit.
		/// The initialization mode depends on the setting of state type.
		void initWithPerUnitParam(
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd, Real Rkq1, Real Llkq1,
			Real Rkq2, Real Llkq2,
			Real H);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
		/// Function parameters have to be given in real units.
		void init(Real om, Real dt,
			Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle);

		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void initStatesInPerUnit(Real initActivePower, Real initReactivePower,
			Real initTerminalVolt, Real initVoltAngle);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector.
		void step(SystemModel& system, Real fieldVoltage, Real mechPower, Real time);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower, Real time, NumericalMethod numMethod);

		/// Retrieves calculated voltage from simulation for next step
		void postStep(SystemModel& system);

		/// Park transform as described in Krause
		Matrix parkTransform(Real theta, double a, double b, double c);
		//Matrix parkTransform(Real theta, Matrix& in);

		/// Inverse Park transform as described in Krause
		Matrix inverseParkTransform(Real theta, double q, double d, double zero);
		//Matrix inverseParkTransform(Real theta, Matrix& in);

		Matrix& getVoltages() { return mVoltageVector; }
		Matrix& getCurrents() { return mCurrentVector; }
		//Matrix& getFluxes() { return mFluxes; }
		double getElectricalTorque() { return mElecTorque; }
		double getRotationalSpeed() { return mOmMech; }
		double getRotorPosition() { return mThetaMech; }

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time) { }
	};
}
