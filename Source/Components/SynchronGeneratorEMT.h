/** Synchron generator (EMT)
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

	class SynchronGeneratorEMT : public BaseComponent {
	protected:

		Logger* mLog;

		// ### Machine parameters ###
		/// nominal power Pn [VA]
		Real mNomPower;
		/// nominal voltage Vn [V] (RMS)
		Real mNomVolt;
		/// nominal frequency fn [Hz]
		Real mNomFreq;
		/// nominal field current Ifn [A]
		Real mNomFieldCur;

		/// stator resistance Rs[Ohm]
		Real mRs;
		/// leakage inductance Ll [H]
		Real mLl;
		/// d-axis mutual inductance Lmd [H]
		Real mLmd;
		/// unsaturated d-axis mutual inductance Lmd [H]
		Real mLmd0;
		/// q-axis mutual inductance Lmq [H]
		Real mLmq;
		/// unsaturated q-axis mutual inductance Lmq [H]
		Real mLmq0;
		/// field resistance Rfd [Ohm]
		Real mRfd;
		/// field leakage inductance Llfd [H]
		Real mLlfd;
		/// d-axis damper resistance Rkd [Ohm]
		Real mRkd;
		/// d-axis damper leakage inductance Llkd [H]
		Real mLlkd;
		/// q-axis damper resistance 1 Rkq1 [Ohm]
		Real mRkq1;
		/// q-axis damper leakage inductance 1 Llkq1 [H]
		Real mLlkq1;
		/// q-axis damper resistance 2 Rkq2 [Ohm]
		Real mRkq2;
		/// q-axis damper leakage inductance 2 Llkq2 [H]
		Real mLlkq2;
		/// q winding inductance
		Real mLaq;
		/// d winding inductance
		Real mLad;

		/// Determinant of Ld (inductance matrix of d axis)
		Real detLd;
		/// Determinant of Lq (inductance matrix of q axis)
		Real detLq;

		/// inertia J [kg*m^2]
		Real mJ;
		/// number of poles
		Int mPoleNumber;
		/// inertia coefficient H
		Real mH;

		// ### Stator base values ###
		/// specifies if the machine parameters are transformed to per unit
		SynchGenStateType mStateType;
		/// base stator voltage
		Real mBase_v;
		/// base stator voltage RMS
		Real mBase_V_RMS;
		/// base stator current
		Real mBase_i;
		/// base stator current RMS
		Real mBase_I_RMS;
		/// base stator impedance
		Real mBase_Z;
		/// base electrical angular frequency
		Real mBase_OmElec;
		/// base mechanical angular frequency
		Real mBase_OmMech;
		/// base stator inductance
		Real mBase_L;
		/// base torque
		Real mBase_T;
		/// base flux linkage
		Real mBase_Psi;

		/// base field current
		Real mBase_ifd;
		/// base field voltage
		Real mBase_vfd;
		/// base field impedance
		Real mBase_Zfd;
		/// base field inductance
		Real mBase_Lfd;


		// ### State variables ###
		/// rotor speed omega_r
		Real mOmMech;
		/// theta
		Real mThetaMech;
		Real mThetaMech2;
		/// mechanical Power Pm [W]
		Real mMechPower;
		/// mechanical torque
		Real mMechTorque;
		/// electrical torque
		Real mElecTorque;

		/// mechanical torque
		Real mMechTorque_past;
		/// electrical torque
		Real mElecTorque_past;
		/// rotor speed omega_r
		Real mOmMech_past;

		/// voltage vector q d 0 kq1 kq2 df kd
		Matrix mVoltages2 = Matrix::Zero(7, 1);
		/// flux linkage vector
		Matrix mFluxes2 = Matrix::Zero(7, 1);
		/// current vector
		Matrix mCurrents2 = Matrix::Zero(7, 1);

		/// voltage vector q d 0 fd kd kq1 kq2
		Real mVd;
		Real mVq;
		Real mV0;
		Real mVfd;
		Real mVkd;
		Real mVkq1;
		Real mVkq2;

		/// current vector q d 0 fd kd kq1 kq2
		Real mId;
		Real mIq;
		Real mI0;
		Real mIfd;
		Real mIkd;
		Real mIkq1;
		Real mIkq2;

		Real mId_past;
		Real mIq_past;

		/// flux linkage vector q d 0 fd kd kq1 kq2
		Real mPsid;
		Real mPsiq;
		Real mPsi0;
		Real mPsifd;
		Real mPsikd;
		Real mPsikq1;
		Real mPsikq2;

		Real mPsid_past;
		Real mPsiq_past;

		/// Interface voltage vector
		Real mVa;
		Real mVb;
		Real mVc;

		/// Interface voltage vector
		Real mIa;
		Real mIb;
		Real mIc;

		/// Number of damping windings in q
		Int DampingWindings;

		// ### Useful Matrices ###
		/// inductance matrix
		Matrix mInductanceMat = Matrix::Zero(7, 7);
		/// resistance matrix
		Matrix mResistanceMat = Matrix::Zero(7, 7);
		/// reactance matrix
		Matrix mReactanceMat = Matrix::Zero(7, 7);
		/// omega - flux matrix
		Matrix mOmegaFluxMat = Matrix::Zero(7, 7);
		/// matrix for reversing stator current directions in calculations with respect to other currents
		Matrix mReverseCurrents = Matrix::Zero(7, 7);


	public:
		SynchronGeneratorEMT() { };
		~SynchronGeneratorEMT();

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		SynchronGeneratorEMT(std::string name, Int node1, Int node2, Int node3,
			Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia, bool logActive = false);

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
			Real initActivePower, Real initReactivePower, Real initTerminalVolt,
			Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void initStatesInPerUnit(Real initActivePower, Real initReactivePower, Real initTerminalVolt,
			Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector.
		void step(SystemModel& system, Real time);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod);

		/// Retrieves calculated voltage from simulation for next step
		void postStep(SystemModel& system);

		/// Park transform as described in Krause
		//Matrix parkTransform(Real theta, Matrix& in);
		Matrix parkTransform2(Real theta, Real a, Real b, Real c);

		/// Inverse Park transform as described in Krause
		//Matrix inverseParkTransform(Real theta, Matrix& in);
		Matrix inverseParkTransform2(Real theta, Real d, Real q, Real zero);

		Matrix& getVoltages() { return mVoltages2; }
		Matrix& getCurrents() { return mCurrents2; }
		Matrix& getFluxes() { return mFluxes2; }
		Real getElectricalTorque() { return mElecTorque*mBase_T; }
		Real getRotationalSpeed() { return mOmMech*mBase_OmMech; }
		Real getRotorPosition() { return mThetaMech; }


		// Methods for network integrated components
		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system) { }
	};
}
