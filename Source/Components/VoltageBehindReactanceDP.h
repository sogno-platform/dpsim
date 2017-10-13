/** Voltage behind reactance (DP)
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

	class VoltageBehindReactanceDP : public BaseComponent {
	protected:

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

		/// d dynamic inductance
		Real mDLmd;
		/// q dynamic inductance
		Real mDLmq;

		Real mLS;
		Real mLM;


		/// d dynamic flux
		Real mDPsid;
		/// q dynamic flux
		Real mDPsiq;

		/// Dynamic d voltage
		Real mDVq;
		/// Dynamic q voltage
		Real mDVd;

		/// Dynamic voltage phase a
		Real mDVaRe;
		Real mDVaIm;
		/// Dynamic voltage phase b
		Real mDVbRe;
		Real mDVbIm;
		/// Dynamic voltage phase c
		Real mDVcRe;
		Real mDVcIm;

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
		Real mOmMech_hist;
		/// theta
		Real mThetaMech;
		Real mThetaMech2;
		Real mThetaMech2_hist;
		Real mTheta0;
		/// mechanical Power Pm [W]
		Real mMechPower;
		/// mechanical torque
		Real mMechTorque;
		/// electrical torque
		Real mElecTorque;
		Real mElecTorque_hist;

		///Number of damping windings in q
		Real DampingWinding = 2;


		Matrix R_load = Matrix::Zero(6, 6);

		Matrix L_constant = Matrix::Zero(3, 3);
		Matrix L_cp = Matrix::Zero(6, 6);
		Matrix R_cp = Matrix::Zero(6, 6);

		/// voltage vector q d 0 kq1 kq2 df kd
		Matrix mVoltages2 = Matrix::Zero(7, 1);
		/// flux linkage vector
		Matrix mFluxes = Matrix::Zero(7, 1);
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

		/// flux linkage vector q d 0 fd kd kq1 kq2
		Real mPsid;
		Real mPsiq;
		Real mPsi0;
		Real mPsifd;
		Real mPsikd;
		Real mPsikq1;
		Real mPsikq2;

		/// Interface voltage vector
		Real mVaRe;
		Real mVbRe;
		Real mVcRe;
		Real mVaIm;
		Real mVbIm;
		Real mVcIm;

		/// Interface voltage vector
		Real mIaRe;
		Real mIbRe;
		Real mIcRe;
		Real mIaIm;
		Real mIbIm;
		Real mIcIm;

		/// Magnetizing flux linkage in qd axes
		Real mPsimq;
		Real mPsimd;


		// Auxiliar Matrix for DP
		MatrixComp LD02 = MatrixComp::Zero(3, 3);
		Matrix LD0 = Matrix::Zero(3, 3);
		MatrixComp LD1 = MatrixComp::Zero(3, 3);
		MatrixComp A1 = MatrixComp::Zero(3, 3);
		MatrixComp A3 = MatrixComp::Zero(3, 3);
		MatrixComp A4 = MatrixComp::Zero(3, 3);
		MatrixComp B1 = MatrixComp::Zero(3, 3);
		MatrixComp A2 = MatrixComp::Zero(3, 3);
		MatrixComp B2 = MatrixComp::Zero(3, 3);
		Matrix L_VP_SFA = Matrix::Zero(6, 6);
		Matrix R_VP_SFA = Matrix::Zero(6, 6);
		Matrix K1 = Matrix::Zero(6, 6);
		Matrix K2 = Matrix::Zero(6, 6);
		Complex alpha;

		// ### Useful Matrices ###
		/// inductance matrix
		Matrix mDInductanceMat = Matrix::Zero(3, 3);
		/// resistance matrix
		Matrix mResistanceMat = Matrix::Zero(3, 3);
		/// reactance matrix
		//Matrix mReactanceMat = Matrix::Zero(7, 7);
		/// omega - flux matrix
		//Matrix mOmegaFluxMat = Matrix::Zero(7, 7);
		/// matrix for reversing stator current directions in calculations with respect to other currents
		//Matrix mReverseCurrents = Matrix::Zero(7, 7);

		Real mLa;
		Real mLb;

		//Historical term of voltage
		Matrix mV_hist = Matrix::Zero(3, 1);

		//Phase Voltages in pu
		Matrix mVabcRe = Matrix::Zero(3, 1);
		Matrix mVabcIm = Matrix::Zero(3, 1);
		Matrix mVabc = Matrix::Zero(6, 1);


		//Historical term of current
		Matrix mIabcRe = Matrix::Zero(3, 1);
		Matrix mIabcIm = Matrix::Zero(3, 1);
		Matrix mIabc = Matrix::Zero(6, 1);

		//Historical term of voltage
		Matrix mDVabcRe = Matrix::Zero(3, 1);
		Matrix mDVabcIm = Matrix::Zero(3, 1);
		Matrix mDVabc = Matrix::Zero(6, 1);

		//Phase Voltages
		Matrix mVoltageVector = Matrix::Zero(6, 1);

		//Phase Currents
		Matrix mCurrentVector = Matrix::Zero(6, 1);


		/// Matrix paremeters for integration of rotor flux linkages - A
		Matrix A_flux = Matrix::Zero(4, 4);
		/// Variables for integration of rotor flux linkages - B
		Matrix B_flux = Matrix::Zero(4, 2);
		/// Variables for integration of rotor flux linkages - C
		Matrix C_flux = Matrix::Zero(4, 1);

		Matrix mRotorFlux = Matrix::Zero(4, 1);
		Matrix mDqStatorCurrents = Matrix::Zero(2, 1);
		Matrix mDqStatorCurrents_hist = Matrix::Zero(2, 1);


	public:
		VoltageBehindReactanceDP() { };

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		VoltageBehindReactanceDP(std::string name, Int node1, Int node2, Int node3,
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

		//void FormTheveninEquivalent(Real dt);
		void CalculateLandR(Real theta, Real omega_s, Real omega);
		void CalculateLandR(Real theta, Real omega_s, Real omega, Real time);
		void CalculateLandR(Real dt, Real time);

		/// Retrieves calculated voltage from simulation for next step
		void postStep(SystemModel& system);

		/// abc to dq
		Matrix abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm);

		/// dq to abc
		Matrix dq0ToAbcTransform(Real theta, Real d, Real q, Real zero);


		Matrix& getVoltages() { return mVoltageVector; }
		Matrix& getCurrents() { return mCurrentVector; }
		//Matrix& getFluxes() { return mFluxes; }
		Real getElectricalTorque() { return mElecTorque; }
		Real getRotationalSpeed() { return mOmMech; }
		Real getRotorPosition() { return mThetaMech2; }

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time) { }
	};
}
