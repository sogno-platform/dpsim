/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/Base/Base_SynchronGenerator.h>
#include <dpsim-models/Base/Base_Exciter.h>
#include <dpsim-models/Signal/TurbineGovernor.h>
#include <dpsim-models/EMT/EMT_Ph1_VoltageSource.h>
#include <dpsim-models/EMT/EMT_Ph1_Resistor.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability
	class SynchronGeneratorVBR :
		public MNASimPowerComp<Real>,
		public Base::SynchronGenerator,
		public MNAVariableCompInterface,
		public SharedFactory<SynchronGeneratorVBR> {
	protected:
		/// d dynamic inductance
		Real mDLmd;
		/// q dynamic inductance
		Real mDLmq;

		/// Auxiliar inductance
		Real mLa;
		/// Auxiliar inductance
		Real mLb;

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

		/// Interface voltage q component
		Real mVq;
		/// Interface voltage d component
		Real mVd;
		/// Interface voltage 0 component
		Real mV0;

		/// Interface current q component
		Real mIq;
		/// Interface current d component
		Real mId;
		/// Interface current 0 component
		Real mI0;

		/// Magnetizing flux linkage 1st damper winding q axis
		Real mPsikq1;
		/// Magnetizing flux linkage 2nd damper winding q axis
		Real mPsikq2;
		/// Magnetizing flux linkage damper winding d axis
		Real mPsikd;
		/// Magnetizing flux linkage excitation
		Real mPsifd;
		/// Magnetizing flux linkage in q axis
		Real mPsimq;
		/// Magnetizing flux linkage in d axis
		Real mPsimd;

		/// Voltage excitation
		Real mVfd;
		
		/// Phase currents in pu
		Matrix mIabc = Matrix::Zero(3, 1);
		///Phase Voltages in pu
		Matrix mVabc = Matrix::Zero(3, 1);
		/// Subtransient voltage in pu
		Matrix mDVabc = Matrix::Zero(3, 1);

		/// Dq stator current vector
		Matrix mDqStatorCurrents = Matrix::Zero(2, 1);
		/// Q axis stator current of  from last time step
		Real mIq_hist;
		/// D axis stator current of  from last time step
		Real mId_hist;

		// ### Useful Matrices ###
		/// inductance matrix
		Matrix mDInductanceMat = Matrix::Zero(3, 3);

		/// Q axis Rotor flux
		Matrix mPsikq1kq2 = Matrix::Zero(2, 1);
		/// D axis rotor flux
		Matrix mPsifdkd = Matrix::Zero(2, 1);
		/// Equivalent Stator Conductance Matrix
		Matrix mConductanceMat = Matrix::Zero(3, 3);
		/// Equivalent Stator Current Source
		Matrix mISourceEq = Matrix::Zero(3, 1);
		/// Dynamic Voltage Vector
		Matrix mDVqd = Matrix::Zero(2, 1);
		/// Equivalent VBR Stator Resistance
		MatrixFixedSize<3, 3> R_eq_vbr = MatrixFixedSize<3, 3>::Zero(3, 3);
		/// Equivalent VBR Stator Voltage Source
		Matrix E_eq_vbr = Matrix::Zero(3, 1);
		/// Park Transformation Matrix
		MatrixFixedSize<3, 3> mKrs_teta = MatrixFixedSize<3, 3>::Zero(3, 3);
		/// Inverse Park Transformation Matrix
		MatrixFixedSize<3, 3> mKrs_teta_inv = MatrixFixedSize<3, 3>::Zero(3, 3);

		/// Auxiliar variables
		Real c21_omega;
		Real c22_omega;
		Real c13_omega;
		Real c14_omega;
		MatrixFixedSize<2, 2> K1a = MatrixFixedSize<2, 2>::Zero(2, 2);
		Matrix K1b = Matrix::Zero(2, 1);
		Matrix K1 = Matrix::Zero(2, 1);
		MatrixFixedSize<2, 2> K2a = MatrixFixedSize<2, 2>::Zero(2, 2);
		Matrix K2b = Matrix::Zero(2, 1);
		Matrix K2 = Matrix::Zero(2, 1);
		Matrix H_qdr = Matrix::Zero(3, 1);
		Matrix h_qdr;
		MatrixFixedSize<3, 3> K = MatrixFixedSize<3, 3>::Zero(3, 3);
		Matrix mEsh_vbr = Matrix::Zero(3, 1);
		Matrix E_r_vbr = Matrix::Zero(3, 1);
		MatrixFixedSize<2, 2> K1K2 = MatrixFixedSize<2, 2>::Zero(2, 2);

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

		MatrixFixedSize<2, 2> Ea = MatrixFixedSize<2, 2>::Zero(2, 2);
		Matrix E1b = Matrix::Zero(2, 1);
		// CHECK: E1 and F1 seem to be vectors (Zero(2, 1)) and not matrices
		Matrix E1 = Matrix::Zero(2, 2);
		MatrixFixedSize<2, 2> Fa = MatrixFixedSize<2, 2>::Zero(2, 2);
		Matrix F1b = Matrix::Zero(2, 1);
		Matrix F1 = Matrix::Zero(2, 2);
		MatrixFixedSize<2, 2> E2b = MatrixFixedSize<2, 2>::Zero(2, 2);
		MatrixFixedSize<2, 2> E2 = MatrixFixedSize<2, 2>::Zero(2, 2);
		MatrixFixedSize<2, 2> F2b = MatrixFixedSize<2, 2>::Zero(2, 2);
		MatrixFixedSize<2, 2> F2 = MatrixFixedSize<2, 2>::Zero(2, 2);
		Matrix F3b = Matrix::Zero(2, 1);
		Matrix F3 = Matrix::Zero(2, 2);
		Matrix C26 = Matrix::Zero(2, 1);


	public:
		/// Defines UID, name and logging level
		SynchronGeneratorVBR(String name, String uid, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		SynchronGeneratorVBR(String name, Logger::Level logLevel = Logger::Level::off);

		/// Initializes the base and fundamental machine parameters in per unit
		void setBaseAndFundamentalPerUnitParameters(Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia);

		/// Initializes the base and operational machine parameters in per unit.
		/// The fundamental machine parameters in per unit are calculated and set accordingly.
		void setBaseAndOperationalPerUnitParameters(Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
			Real Rs, Real Ld, Real Lq, Real Ld_t, Real Lq_t, Real Ld_s, Real Lq_s,
			Real Ll, Real Td0_t, Real Tq0_t, Real Td0_s, Real Tq0_s, Real inertia);

		/// Initialize states according to desired initial electrical powerflow and mechanical input power
		void setInitialValues(Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle, Real initMechPower);

		/// Initialize components with correct network frequencies
		void initialize(Matrix frequencies) override;

		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
		/// Function parameters have to be given in real units.
		void initialize(Real om, Real dt);

		/// Initializes internal variables of the component
		void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit();

		/// Park transform as described in Krause
		Matrix parkTransform(Real theta, Real a, Real b, Real c);

		/// Inverse Park transform as described in Krause
		Matrix inverseParkTransform(Real theta, Real q, Real d, Real zero);

		/// Calculate inductance Matrix L and its derivative
		void CalculateL();
		void CalculateAuxiliarConstants(Real dt);
		void CalculateAuxiliarVariables();

		//Matrix& rotorFluxes() { return mRotorFlux; }
		Matrix& dqStatorCurrents();
		Real electricalTorque() const;
		Real rotationalSpeed() const;
		Real rotorPosition() const;
		Matrix& statorCurrents();

		// #### MNA section ####
		/// Stamps system matrix
		void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix);
		/// Stamps right side (source) vector
		void mnaCompApplyRightSideVectorStamp(Matrix& rightVector);
		/// MNA pre step operations
		void mnaCompPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);
		/// Mark that parameter changes so that system matrix is updated
		Bool hasParameterChanged() override;
	};
}
}
}
