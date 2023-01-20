/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Signal/ExciterDC1Simp.h>
#include <dpsim-models/Signal/TurbineGovernor.h>

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

		/// Add governor and turbine
		void addGovernor(Real Ta, Real Tb, Real Tc, Real Fa,
			Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef);
		/// Add automatic voltage regulator
			void addExciter(std::shared_ptr<Base::Exciter> exciter);

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
		/// Flag to remember when initial values are set
		Bool mInitialValuesSet = false;

		// ### Machine parameters ###
		/// nominal power Pn [VA]
		Real mNomPower = 0;
		/// nominal voltage Vn [V] (phase-to-phase RMS)
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


		/// d-axis mutual inductance Lmd [H]
		Real mLmd = 0;
		/// q-axis mutual inductance Lmq [H]
		Real mLmq = 0;
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

	public:
		/// stator resistance Rs [Ohm]
		const Attribute<Real>::Ptr mRs;
		/// leakage inductance Ll [H]
		const Attribute<Real>::Ptr mLl;
		/// d-axis inductance Ld [H]
		const Attribute<Real>::Ptr mLd;
		/// q-axis inductance Lq [H]
		const Attribute<Real>::Ptr mLq;

		// Operational parameters
		/// Transient d-axis inductance [H]
		const Attribute<Real>::Ptr mLd_t;
		/// Transient q-axis inductance [H]
		const Attribute<Real>::Ptr mLq_t;
		/// Subtransient d-axis inductance [H]
		const Attribute<Real>::Ptr mLd_s;
		/// Subtransient q-axis inductance [H]
		const Attribute<Real>::Ptr mLq_s;
		/// Transient time constant of d-axis [s]
		const Attribute<Real>::Ptr mTd0_t;
		/// Transient time constant of q-axis [s]
		const Attribute<Real>::Ptr mTq0_t;
		/// Subtransient time constant of d-axis [s]
		const Attribute<Real>::Ptr mTd0_s;
		/// Subtransient time constant of q-axis [s]
		const Attribute<Real>::Ptr mTq0_s;

	protected:
		// #### Initial Values ####
		Complex mInitElecPower = 0;
		Complex mInitTermVoltage = 0;
		Real mInitMechPower = 0;

		// ### Stator base values ###
		/// base stator voltage (phase-to-ground peak)
		Real mBase_V  = 0;
		/// base stator voltage (phase-to-ground RMS)
		Real mBase_V_RMS = 0;
		/// base stator current peak
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
		/// theta
		Real mThetaMech = 0;
	public:
		/// rotor angle delta
		const Attribute<Real>::Ptr mDelta;
		/// mechanical torque
		const Attribute<Real>::Ptr mMechTorque;
		/// inertia constant H [s] for per unit or moment of inertia J [kg*m^2]
		const Attribute<Real>::Ptr mInertia;
		/// rotor speed omega_r
		const Attribute<Real>::Ptr mOmMech;
		/// Active part of the electrical power
		const Attribute<Real>::Ptr mElecActivePower;
		/// Reactive part of the electrical power
		const Attribute<Real>::Ptr mElecReactivePower;
		/// mechanical Power Pm [W]
		const Attribute<Real>::Ptr mMechPower;
		/// electrical torque
		const Attribute<Real>::Ptr mElecTorque;
		/// Voltage excitation
		const Attribute<Real>::Ptr mVfd;

	protected:
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

		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void initPerUnitStates();

		// #### Controllers ####
		/// Determines if Turbine and Governor are activated
		Bool mHasTurbineGovernor = false;
		/// Determines if Exciter is activated
		Bool mHasExciter = false;

		// Deprecated
		Real mInitTerminalVoltage = 0;
		Real mInitVoltAngle = 0;

		/// Constructor
		explicit SynchronGenerator(CPS::AttributeBase::Map &attributeList) :
			mRs(Attribute<Real>::create("Rs", attributeList, 0)),
			mLl(Attribute<Real>::create("Ll", attributeList, 0)),
			mLd(Attribute<Real>::create("Ld", attributeList, 0)),
			mLq(Attribute<Real>::create("Lq", attributeList, 0)),
			mLd_t(Attribute<Real>::create("Ld_t", attributeList, 0)),
			mLq_t(Attribute<Real>::create("Lq_t", attributeList, 0)),
			mLd_s(Attribute<Real>::create("Ld_s", attributeList, 0)),
			mLq_s(Attribute<Real>::create("Lq_s", attributeList, 0)),
			mTd0_t(Attribute<Real>::create("Td0_t", attributeList, 0)),
			mTq0_t(Attribute<Real>::create("Tq0_t", attributeList, 0)),
			mTd0_s(Attribute<Real>::create("Td0_s", attributeList, 0)),
			mTq0_s(Attribute<Real>::create("Tq0_s", attributeList, 0)),
			mDelta(Attribute<Real>::create("delta_r", attributeList, 0)),
			mMechTorque(Attribute<Real>::create("T_m", attributeList, 0)),
			mInertia(Attribute<Real>::create("inertia", attributeList, 0)),
			mOmMech(Attribute<Real>::create("w_r", attributeList, 0)),
			mElecActivePower(Attribute<Real>::create("P_elec", attributeList, 0)),
			mElecReactivePower(Attribute<Real>::create("Q_elec", attributeList, 0)),
			mMechPower(Attribute<Real>::create("P_mech", attributeList, 0)),
			mElecTorque(Attribute<Real>::create("T_e", attributeList, 0)),
			mVfd(Attribute<Real>::create("Vfd", attributeList, 0)) { };

		///
		void setBaseParameters(Real nomPower, Real nomVolt, Real nomFreq);
		///
		void setBaseParameters(Real nomPower, Real nomVolt, Real nomFreq, Real nomFieldCur);

		///
		void calcStateSpaceMatrixDQ();
		///
		Real calcHfromJ(Real J, Real omegaNominal, Int polePairNumber);

	public:
		/// Destructor - does nothing.
		virtual ~SynchronGenerator() { }

		/// Initializes the base and fundamental machine parameters in per unit
		void setBaseAndFundamentalPerUnitParameters(
			Real nomPower, Real nomVolt, Real nomFreq, Real nomFieldCur,
			Int poleNumber, Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd,
			Real Rkd, Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia);

		/// Initializes the base and operational machine parameters in per unit.
		/// The fundamental machine parameters in per unit are calculated and set accordingly.
		void setBaseAndOperationalPerUnitParameters(
			Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ld, Real Lq, Real Ld_t, Real Lq_t, Real Ld_s, Real Lq_s,
			Real Ll, Real Td0_t, Real Tq0_t, Real Td0_s, Real Tq0_s, Real inertia);

		///
		void setFundamentalPerUnitParameters(Int poleNumber,
			Real Rs, Real Ll, Real Lmd, Real Lmq,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd, Real Rkq1, Real Llkq1,
			Real Rkq2, Real Llkq2,
			Real inertia);

		///
		void applyFundamentalPerUnitParameters();

		///
		void setAndApplyFundamentalPerUnitParameters(
			Int poleNumber, Real Rs, Real Ll, Real Lmd, Real Lmq,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia);

		///
		void setOperationalPerUnitParameters(
			Int poleNumber, Real inertia,
			Real Rs, Real Ld, Real Lq, Real Ll,
			Real Ld_t, Real Lq_t, Real Ld_s, Real Lq_s,
			Real Td0_t, Real Tq0_t, Real Td0_s, Real Tq0_s);

		///
		void calculateFundamentalFromOperationalParameters();

		///
		void setInitialValues(
			Real initActivePower, Real initReactivePower,
			Real initTerminalVolt, Real initVoltAngle, Real initMechPower);

		/// Switch to determine the integration method for the machine model.
		void setNumericalMethod(NumericalMethod method) { mNumericalMethod = method; }

		/// Signal component modelling governor control and steam turbine
		std::shared_ptr<Signal::TurbineGovernor> mTurbineGovernor;
		/// Signal component modelling voltage regulator and exciter
		std::shared_ptr<Base::Exciter> mExciter;
	};
}
}
