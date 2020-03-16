/**
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

#include <cps/Definitions.h>
#include <cps/Signal/Exciter.h>
#include <cps/Signal/TurbineGovernor.h>

namespace CPS {
namespace Base {
	/// @brief Base synchronous generator model
	///
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability
	class SynchronGenerator {
	public:
		/// \brief State type of machine variables.
		///
		/// Determines if the machine states are considered in per unit,
		/// referred to the stator parameters or the rotor parameters.
		enum class StateType { perUnit, statorReferred, rotorReferred };
		/// \brief Machine parameters type.
		enum class ParameterType { perUnit, statorReferred, operational };
	protected:
		///
		NumericalMethod mNumericalMethod; //not needed if sundials used; could instead determine implicit / explicit solve
		/// Simulation angular system speed
		Real mSystemOmega;
		/// Simulation time step
		Real mTimeStep;
		/// specifies if the machine parameters are transformed to per unit
		StateType mStateType = StateType::perUnit;
		///
		ParameterType mParameterType;

		// ### Machine parameters ###
		/// nominal power Pn [VA]
		Real mNomPower = 0;
		/// nominal voltage Vn [V] (RMS)
		Real mNomVolt = 0;
		/// nominal frequency fn [Hz]
		Real mNomFreq = 0;
		/// nominal angular frequency wn [Hz]
		Real mNomOmega = 0;
		/// nominal field current Ifn [A]
		Real mNomFieldCur = 0;
		/// Number of damping windings in q
		Int mNumDampingWindings = 0;
		/// mNumber of poles
		Int mPoleNumber = 0;
		/// inertia constant H [s] for per unit or moment of inertia J [kg*m^2]
		Real mInertia = 0;

		/// stator resistance Rs [Ohm]
		Real mRs = 0;
		/// leakage inductance Ll [H]
		Real mLl = 0;
		/// d-axis mutual inductance Lmd [H]
		Real mLmd = 0;
		/// d-axis inductance Ld [H]
		Real mLd = 0;
		/// q-axis mutual inductance Lmq [H]
		Real mLmq = 0;
		/// q-axis inductance Lq [H]
		Real mLq = 0;
		/// field resistance Rfd [Ohm]
		Real mRfd = 0;
		/// field leakage inductance Llfd [H]
		Real mLlfd = 0;
		/// field inductance Lfd [H]
		Real mLfd = 0;
		/// field winding inductance Lf [H]
		Real mLf = 0;
		/// d-axis damper resistance Rkd [Ohm]
		Real mRkd = 0;
		/// d-axis damper leakage inductance Llkd [H]
		Real mLlkd = 0;
		/// d-axis damper inductance Lkd [H]
		Real mLkd = 0;
		/// q-axis damper resistance 1 Rkq1 [Ohm]
		Real mRkq1 = 0;
		/// q-axis damper leakage inductance 1 Llkq1 [H]
		Real mLlkq1 = 0;
		/// q-axis damper inductance 1 Lkq1 [H]
		Real mLkq1 = 0;
		/// q-axis damper resistance 2 Rkq2 [Ohm]
		Real mRkq2 = 0;
		/// q-axis damper leakage inductance 2 Llkq2 [H]
		Real mLlkq2 = 0;
		/// q-axis damper inductance 2 Lkq2 [H]
		Real mLkq2 = 0;

		// #### Initial Values ####
		Complex mInitElecPower = 0;
		Complex mInitTermVoltage = 0;
		Real mInitFieldVoltage = 0;
		Real mInitMechPower = 0;

		// ### Stator base values ###
		/// base stator voltage
		Real mBase_V  = 0;
		/// base stator voltage RMS
		Real mBase_V_RMS = 0;
		/// base stator current
		Real mBase_I = 0;
		/// base stator current RMS
		Real mBase_I_RMS = 0;
		/// base stator impedance
		Real mBase_Z = 0;
		/// base electrical angular frequency
		Real mBase_OmElec = 0;
		/// base mechanical angular frequency
		Real mBase_OmMech = 0;
		/// base stator inductance
		Real mBase_L = 0;
		/// base torque
		Real mBase_T = 0;
		/// base flux linkage
		Real mBase_Psi = 0;

		/// base field current
		Real mBase_ifd = 0;
		/// base field voltage
		Real mBase_vfd = 0;
		/// base field impedance
		Real mBase_Zfd = 0;
		/// base field inductance
		Real mBase_Lfd = 0;

		// ### Useful Matrices ### (still needed? )
		/// Inductance matrix which is numerically equal to the reactance matrix in per unit
		Matrix mInductanceMat;
		/// resistance matrix
		Matrix mResistanceMat;
		/// Inverse of the inductance matrix
		Matrix mInvInductanceMat;

		// ### State variables ###
		/// rotor speed omega_r
		Real mOmMech = 0;
		/// theta
		Real mThetaMech = 0;
		/// mechanical Power Pm [W]
		Real mMechPower = 0;
		/// mechanical torque
		Real mMechTorque = 0;
		/// Active part of the electrical power
		Real mElecActivePower = 0;
		/// electrical torque
		Real mElecTorque = 0;
		/// \brief Vector of stator and rotor voltages.
		///
		/// v_d - Stator voltage in d axis \n
		/// v_fd - Rotor voltage field winding \n
		/// v_kd - Rotor voltage damping winding in d axis \n
		/// v_q - Stator voltage in q axis \n
		/// v_kq1 - Rotor voltage damping winding 1 in q axis \n
		/// v_kq2 - Rotor voltage damping winding 2 in q axis \n
		/// v_0 - Stator voltage 0 component \n
		Matrix mVsr;
		/// \brief Vector of stator and rotor currents.
		///
		/// i_d - stator current in d axis
		/// i_fd - Rotor current field winding
		/// i_kd - Rotor current damping winding in d axis
		/// i_q - stator current in q axis
		/// i_kq1 - Rotor current damping winding 1 in q axis
		/// i_kq2 - Rotor current damping winding 2 in q axis
		/// i_0 - stator current 0 component
		Matrix mIsr;
		/// \brief Vector of stator and rotor fluxes.
		///
		/// psi_d - stator flux linkage in d axis
		/// psi_fd - rotor flux linkage in field winding
		/// psi_kd - rotor flux linkage in damping winding from d axis
		/// psi_q - stator flux linkage in q axis
		/// psi_kq1 - rotor flux linkage in damping winding 1 from q axis
		/// psi_kq2 - rotor flux linkage in damping winding 2 from q axis
		/// psi_0 - stator flux linkage 0 component
		Matrix mPsisr; //equivalent to Fluxes
		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit.
		/// The initialization mode depends on the setting of state type.

		// #### dq-frame specific variables ####
		/// dq0 voltage calculated from terminal voltage
		Matrix mVdq0;
		/// dq0 current calculated from terminal current
		Matrix mIdq0;
		/// Flux state space matrix excluding omega term
		Matrix mFluxStateSpaceMat;
		/// Omega-flux matrix for state space system
		Matrix mOmegaFluxMat;
		/// Calculates currents from fluxes
		Matrix mFluxToCurrentMat;
		/// Inductance to calculate magnetizing flux linkage from winding flux linkages
		Real mLad;
		/// Inductance to calculate magnetizing flux linkage from winding flux linkages
		Real mLaq;
		/// Determinant of d-axis inductance matrix
		Real mDetLd;
		/// Determinant of q-axis inductance matrix
		Real mDetLq;

		/// Determines if compensation elements are used
		Bool mCompensationOn;
		/// Compensation Resistance
		Real mRcomp;

		void setFundamentalPerUnitParameters(
			Real Rs, Real Ll, Real Lmd, Real Lmq,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd, Real Rkq1, Real Llkq1,
			Real Rkq2, Real Llkq2,
			Real inertia);
		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void initPerUnitStates();

		// #### Controllers - to be removed ####
		/// Determines if Exciter is activated
		Bool mHasExciter = false;
		/// Determines if Turbine and Governor are activated
		Bool mHasTurbineGovernor = false;
		// Deprecated
		Real mInitTerminalVoltage = 0;
		Real mInitVoltAngle = 0;

		/// Constructor - does nothing.
		SynchronGenerator() { }
		///
		void setBaseParameters(Real nomPower, Real nomVolt, Real nomFreq);

		// ### Deprecated ####
		///
		void addExciter(Real Ta, Real Ka, Real Te, Real Ke,
			Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd);
		///
		void addGovernor(Real Ta, Real Tb, Real Tc, Real Fa,
			Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef);

		///
		void calcStateSpaceMatrixDQ();
		///
		Real calcHfromJ(Real J, Real omegaNominal, Int polePairNumber);

	public:
		/// Destructor - does nothing.
		virtual ~SynchronGenerator() { }

		/// \brief Defines UID, name, machine parameters and logging level.
		///
		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		void setBaseAndFundamentalPerUnitParameters(
			Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd,
			Real Rkd, Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia);

		///
		void setInitialValues(
			Real initActivePower, Real initReactivePower,
			Real initTerminalVolt, Real initVoltAngle,
			Real initFieldVoltage, Real initMechPower);

		/// Switch to determine the integration method for the machine model.
		void setNumericalMethod(NumericalMethod method) { mNumericalMethod = method; }
	};
}
}
