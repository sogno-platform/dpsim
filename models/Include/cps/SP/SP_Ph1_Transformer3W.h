/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/PFSolverInterfaceBranch.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/SP/SP_Ph1_Resistor.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Shunt.h>
#include <cps/Base/Base_Ph1_Transformer.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// Transformer that includes an inductance and resistance
	class Transformer3W :
		public SimPowerComp<Complex>,
		public SharedFactory<Transformer3W>,
		public PFSolverInterfaceBranch,
		public MNAInterface,
		public Base::Ph1::Transformer3W {

	private:
		/// Internal inductors to model losses
		std::shared_ptr<SP::Ph1::Inductor> mSubInductor1;
		std::shared_ptr<SP::Ph1::Inductor> mSubInductor2;
		std::shared_ptr<SP::Ph1::Inductor> mSubInductor3;
		/// Internal resistors to model losses
		std::shared_ptr<SP::Ph1::Resistor> mSubResistor1;
		std::shared_ptr<SP::Ph1::Resistor> mSubResistor2;
		std::shared_ptr<SP::Ph1::Resistor> mSubResistor3;
		/// Internal parallel resistance as snubber
		std::shared_ptr<SP::Ph1::Resistor> mSubSnubResistor;

		/// Snubber resistance added on the low voltage side
		Real mSnubberResistance;

		/// Rated Apparent Powers [VA]
		Real mRatedPower1 = 0;
		Real mRatedPower2 = 0;
		Real mRatedPower3 = 0;
        /// Transformer ratios magnitude
		Real mRatioAbs1 = 1;
		Real mRatioAbs2 = 1;
		Real mRatioAbs3 = 1;
        /// Transformer ratios phase [deg]
		Real mRatioPhase1 = 0;
		Real mRatioPhase2 = 0;
		Real mRatioPhase3 = 0;

		/// Nominal omega ??
		Real mNominalOmega;

		/// Voltage [V] ??
		Real mSvVoltage;

		/// Conductances [S]
		Real mConductance1;
		Real mConductance2;
		Real mConductance3;

		/// Reactances [Ohm]
		Real mReactance1;
		Real mReactance2;
		Real mReactance3;
		

		/// Magnetizing reactance [Ohm]
		Real mMagnetizingReactance=1e9;
		/// Leakage
		Complex mLeakage1;
		Complex mLeakage2;
		Complex mLeakage3;
		/// Magnetizing impedance
		Complex mMagnetizing;

		/// base apparent power[VA]
		Real mBaseApparentPower;
		/// base impedance [ohm]
		Real mBaseImpedance;
        /// base inductance [H]
        Real mBaseInductance;
		/// base admittance [S]
		Real mBaseAdmittance;
		///base omega [1/s]
		Real mBaseOmega;
		/// base voltage [V]
		Real mBaseVoltage;
		///base current [A]
		Real mBaseCurrent;

		/// resistance
		Real mResistancePerUnit1;
		/// resistance
		Real mResistancePerUnit2;
		/// resistance
		Real mResistancePerUnit3;
		/// reactance
		Real mReactancePerUnit1;
		/// reactance
		Real mReactancePerUnit2;
		/// reactance
		Real mReactancePerUnit3;

        /// inductance
        Real mInductancePerUnit1;
		/// inductance
        Real mInductancePerUnit2;
		/// inductance
        Real mInductancePerUnit3;
		/// leakage impedance
		Complex mLeakagePerUnit1;
		/// leakage impedance
		Complex mLeakagePerUnit2;
		/// leakage impedance
		Complex mLeakagePerUnit3;
		/// magnetizing impedance
		Complex mMagnetizingPerUnit;

		// #### Admittance matrix stamp ####
		MatrixComp mY_element;

		// #### Power flow results ####
		/// branch Current flow [A]
		Eigen::Matrix<CPS::Complex, 3, 1, Eigen::DontAlign> mCurrent;
		/// branch active powerflow [W], coef(0) has data from node 0, coef(1) from node 1.
		Eigen::Matrix<CPS::Real, 3, 1, Eigen::DontAlign> mActivePowerBranch;
		/// branch reactive powerflow [Var]
		Eigen::Matrix<CPS::Real, 3, 1, Eigen::DontAlign> mReactivePowerBranch;
		/// whether the total power injection of its from node is stored in this line
		Bool mStoreNodalPowerInjection = false;
		/// nodal active power injection
		Real mActivePowerInjection;
		/// nodal reactive power injection
		Real mReactivePowerInjection;

		/// Boolean for considering resistive losses with sub resistor
		Bool mWithResistiveLosses;
	public:
		/// Defines UID, name and logging level
		Transformer3W(String uid, String name,
			Logger::Level logLevel = Logger::Level::off, Bool withResistiveLosses = false);
		/// Defines name and logging level
		Transformer3W(String name, Logger::Level logLevel = Logger::Level::off)
			: Transformer3W(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name) override;

		// #### General ####
		/// Set transformer specific parameters (without rated power)
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real nomVoltageEnd3, 
						   Real ratioAbs1, Real ratioAbs2, Real ratioAbs3, 
						   Real ratioPhase1, Real ratioPhase2, Real ratioPhase3, 
						   Real resistance1, Real resistance2, Real resistance3,
						   Real inductance1, Real inductance2, Real inductance3);
		/// Set transformer specific parameters
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real nomVoltageEnd3,
						   Real ratedPower1, Real ratedPower2, Real ratedPower3,
						   Real ratioAbs1, Real ratioAbs2, Real ratioAbs3, 
						   Real ratioPhase1, Real ratioPhase2, Real ratioPhase3, 
						   Real resistance1, Real resistance2, Real resistance3,
						   Real inductance1, Real inductance2, Real inductance3);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) override;

		// #### Powerflow section ####
		/// Set base voltage
		void setBaseVoltage(Real baseVoltage);
		/// Initializes component from power flow data
		void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
		/// Stamps admittance matrix
		void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) override;
		/// updates branch current and power flow, input pu value, update with real value
		void updateBranchFlow(VectorComp& current, VectorComp& powerflow);
		/// stores nodal injection power in this line object
		void storeNodalInjection(Complex powerInjection);

		// #### Getter ####
		/// get admittance matrix
		MatrixComp Y_element();

		// #### MNA Section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector) override;
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector) override;
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);
		class MnaPostStep : public Task {
		public:
			MnaPostStep(Transformer3W& transformer, Attribute<Matrix>::Ptr leftVector) :
				Task(transformer.mName + ".MnaPostStep"), mTransformer(transformer), mLeftVector(leftVector) {
					mTransformer.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mTransformer.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			Transformer3W& mTransformer;
			Attribute<Matrix>::Ptr mLeftVector;
		};
    };
}
}
}
