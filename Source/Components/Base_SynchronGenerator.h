/** Synchron generator (EMT)
*
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "Component.h"

namespace DPsim {
namespace Components {

	enum class SynchronGeneratorStateType { perUnit, statorReferred };
	enum class SynchronGeneratorParamType { perUnit, statorReferred };

	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability

	class SynchronGeneratorBase : public Component {

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
		/// Number of damping windings in q
		Int mNumDampingWindings;
		/// mNumber of poles
		Int mPoleNumber;
		/// inertia coefficient H
		Real mH;
		/// inertia J [kg*m^2]
		Real mJ;

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

		// ### Stator base values ###
		/// specifies if the machine parameters are transformed to per unit
		SynchronGeneratorStateType mStateType;
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

		// ### Useful Matrices ###
		/// inductance matrix
		Matrix mInductanceMat;
		/// resistance matrix
		Matrix mResistanceMat;

		// ### State variables ###
		/// rotor speed omega_r
		Real mOmMech;
		/// theta
		Real mThetaMech;
		/// mechanical Power Pm [W]
		Real mMechPower;
		/// mechanical torque
		Real mMechTorque;
		/// electrical torque
		Real mElecTorque;

		/// stator voltage in d axis
		Real mVd;
		/// stator voltage in q axis
		Real mVq;
		/// stator voltage 0 component
		Real mV0;
		/// Rotor voltage field winding
		Real mVfd;
		/// Rotor voltage damping winding in d axis
		Real mVkd;
		/// Rotor voltage damping winding 1 in q axis
		Real mVkq1;
		/// Rotor voltage damping winding 2 in q axis
		Real mVkq2;

		/// stator current in d axis
		Real mId;
		/// stator current in q axis
		Real mIq;
		/// stator current 0 component
		Real mI0;
		/// Rotor current field winding
		Real mIfd;
		/// Rotor current damping winding in d axis
		Real mIkd;
		/// Rotor current damping winding 1 in q axis
		Real mIkq1;
		/// Rotor current damping winding 2 in q axis
		Real mIkq2;

		/// stator flux linkage in d axis
		Real mPsid;
		/// stator flux linkage in q axis
		Real mPsiq;
		/// stator flux linkage 0 component
		Real mPsi0;
		/// rotor flux linkage in field winding
		Real mPsifd;
		/// rotor flux linkage in damping winding from d axis
		Real mPsikd;
		/// rotor flux linkage in damping winding 1 from q axis
		Real mPsikq1;
		/// rotor flux linkage in damping winding 2 from q axis
		Real mPsikq2;

	public:
		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		SynchronGeneratorBase(String name, Int node1, Int node2, Int node3,
			Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia, Logger::Level logLevel = Logger::Level::NONE);

		virtual ~SynchronGeneratorBase() { };

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit.
		/// The initialization mode depends on the setting of state type.
		virtual void initWithPerUnitParam(
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd, Real Rkq1, Real Llkq1,
			Real Rkq2, Real Llkq2,
			Real H);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
		/// Function parameters have to be given in real units.
		void initialize(Real om, Real dt,
			Real initActivePower, Real initReactivePower, Real initTerminalVolt,
			Real initVoltAngle, Real initFieldVoltage, Real initMechPower) { };

		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void initStatesInPerUnit(Real initActivePower, Real initReactivePower, Real initTerminalVolt,
			Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

		/// Calculates states for next time step.
		void step(SystemModel& system, Real time) = 0;

		/// Calculates states for next time step in per unit.
		void stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod) { };

		/// Retrieves calculated voltage from simulation for next step.
		void postStep(SystemModel& system) { };

		// Methods for network integrated components
		virtual void initialize(SystemModel& system) { }
		virtual void applySystemMatrixStamp(SystemModel& system) { }
		virtual void applyRightSideVectorStamp(SystemModel& system) { }
	};
}
}
