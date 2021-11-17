/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>
#include <cps/MathUtils.h>

namespace CPS {
namespace Base {
	class SimpSynchronousGenerator {

	protected:
		///
		Real mTimeStep;
		Real mSimTime;

		// ### Base quantities (stator refered) ###
		/// Nominal power
		Real mNomPower;
		/// Nominal voltage
		Real mNomVolt;
		// Nominal frequency
		Real mNomFreq;
		/// Nominal Omega
		Real mNomOmega;
		/// Base voltage RMS
		Real mBase_V_RMS;
		/// Base peak voltage
		Real mBase_V;
		/// Base RMS current
		Real mBase_I_RMS;
		/// Base peak current
		Real mBase_I;
		/// Base impedance
		Real mBase_Z;
		/// Base omega electric
		Real mBase_OmElec;
		/// Base inductance
		Real mBase_L;
		/// Base omega mech
		Real mBase_OmMech;
		/// Inertia 
		Real mH;


		// ### Operational Parameters  (p.u.) ###
		/// d-axis inductance
		Real mLd; 
		/// d-axis inductance
		Real mLq;
		/// 0-axis inductance
		Real mL0;
		/// Subtransient d-axis inductance
		Real mLd_t;
		/// Subtransient q-axis inductance
		Real mLq_t;   
		/// Subtransient d-axis inductance
		Real mLd_s = 0;
		/// Subtransient q-axis inductance
		Real mLq_s = 0;
		/// Transient time constant of d-axis
		Real mTd0_t;
		/// Transient time constant of q-axis
		Real mTq0_t;
		/// Subtransient time constant of d-axis
		Real mTd0_s = 0;
		/// Subtransient time constant of q-axis
		Real mTq0_s = 0;
		

		// ### Initial values ###
		/// Complex interface current
		Complex mIntfCurrentComplex;
		/// Complex interface voltage
		Complex mIntfVoltageComplex;
		/// initial electrical power
		Complex mInitElecPower;
		/// initial mechanical power 
		Real mInitMechPower;
		/// initial terminal voltage phase a (p.u.)
		Complex mInitVoltage;
		/// angle of initial armature voltage
		Real mInitVoltageAngle;
		/// initial armature voltage phase a (p.u.)
		Complex mInitCurrent;
		/// angle of initial armature current
		Real mInitCurrentAngle;
		/// initial field voltage (p.u.)
		Real mEf;


		// ### State variables [p.u.]###
		/// stator electrical torque
		Real mElecTorque;
		/// Mechanical torque
		Real mMechTorque;
		/// Rotor speed
		Real mOmMech;
		/// mechanical system angle (between d-axis and stator a-axis)
		Real mThetaMech;
		/// Load angle (between q-axis and stator a-axis)
		Real mDelta;		

		/// Constructor - does nothing.
		SimpSynchronousGenerator() = default;

	public:
		/// Destructor - does nothing.
		virtual ~SimpSynchronousGenerator() { }

		/// 
		void setBaseParameters(Real nomPower, Real nomVolt, Real nomFreq);
		/// Initialization for 3 Order SynGen
		void setOperationalParametersPerUnit(Real nomPower, 
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Td0_t);
		/// Initialization for 4 Order SynGen
		void setOperationalParametersPerUnit(Real nomPower, 
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t);
		/// Initialization for 6 Order SynGen
		void setOperationalParametersPerUnit(Real nomPower, 
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t,
			Real Ld_s, Real Lq_s, Real Td0_s, Real Tq0_s);
		///
		void setInitialValues(Complex initComplexElectricalPower, 
			Real initMechanicalPower, Complex initTerminalVoltage);
	};
}
}
