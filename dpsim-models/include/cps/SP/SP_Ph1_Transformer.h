/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
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
#include <cps/SP/SP_Ph1_Capacitor.h>
#include <cps/Base/Base_Ph1_Transformer.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// Transformer that includes an inductance and resistance
	class Transformer :
		public SimPowerComp<Complex>,
		public SharedFactory<Transformer>,
		public PFSolverInterfaceBranch,
		public MNAInterface,
		public Base::Ph1::Transformer {

	private:
		/// Internal resistor to model losses
		std::shared_ptr<SP::Ph1::Resistor> mSubResistor;
		/// Internal inductor to model losses
		std::shared_ptr<SP::Ph1::Inductor> mSubInductor;

		/// Internal parallel resistance 1 as snubber
		std::shared_ptr<SP::Ph1::Resistor> mSubSnubResistor1;
		/// Internal parallel resistance 2 as snubber
		std::shared_ptr<SP::Ph1::Resistor> mSubSnubResistor2;
		/// Internal parallel capacitance 1 as snubber
		std::shared_ptr<SP::Ph1::Capacitor> mSubSnubCapacitor1;
		/// Internal parallel capacitance 2 as snubber
		std::shared_ptr<SP::Ph1::Capacitor> mSubSnubCapacitor2;

		/// Snubber resistance 1 [Ohm]
		Real mSnubberResistance1;
		/// Snubber resistance 2 [Ohm]
		Real mSnubberResistance2;
		/// Snubber capacitance 1 [F]
		Real mSnubberCapacitance1;
		/// Snubber capacitance 2 [F]
		Real mSnubberCapacitance2;

        /// Transformer ratio magnitude
		Real mRatioAbs = 1;
        /// Transformer ratio pase [deg]
		Real mRatioPhase = 0;
		/// Nominal omega
		Real mNominalOmega;

		/// Voltage [V]
		/// FIXME: Not used
		Real mSvVoltage;
		/// Conductance [S]
		/// FIXME: Only set, never read
		Real mConductance;
		/// Reactance [Ohm]
		Real mReactance;

		/// Magnetizing reactance [Ohm]
		Real mMagnetizingReactance=1e9;
		/// Leakage
		Complex mLeakage;
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
		
		///base current [A]
		Real mBaseCurrent;

		/// resistance
		Real mResistancePerUnit;
		/// reactance
		Real mReactancePerUnit;
        /// inductance
        Real mInductancePerUnit;
		/// leakage impedance
		Complex mLeakagePerUnit;
		/// magnetizing impedance
		Complex mMagnetizingPerUnit;
        /// transformer ratio
        Real mRatioAbsPerUnit;

		// #### Admittance matrix stamp ####
		MatrixComp mY_element;

		/// Boolean for considering resistive losses with sub resistor
		Bool mWithResistiveLosses;
	public:
		/// base voltage [V]
		const Attribute<Real>::Ptr mBaseVoltage;

		// #### Power flow results ####
		/// branch Current flow [A]
		const Attribute<MatrixComp>::Ptr mCurrent;
		/// CHECK: Are these derived attributes necessary?
		Attribute<CPS::Complex>::Ptr mCurrent_0;
		Attribute<CPS::Complex>::Ptr mCurrent_1;
		
		/// branch active powerflow [W], coef(0) has data from node 0, coef(1) from node 1.
		const Attribute<Matrix>::Ptr mActivePowerBranch;
		/// CHECK: Are these derived attributes necessary?
		Attribute<CPS::Real>::Ptr mActivePowerBranch_0;
		Attribute<CPS::Real>::Ptr mActivePowerBranch_1;

		/// branch reactive powerflow [Var]
		const Attribute<Matrix>::Ptr mReactivePowerBranch;
		/// CHECK: Are these derived attributes necessary?
		Attribute<CPS::Real>::Ptr mReactivePowerBranch_0;
		Attribute<CPS::Real>::Ptr mReactivePowerBranch_1;
		/// nodal active power injection
		const Attribute<Real>::Ptr mActivePowerInjection;
		/// nodal reactive power injection
		const Attribute<Real>::Ptr mReactivePowerInjection;
		/// whether the total power injection of its from node is stored in this line
		/// FIXME: This is only written to, but never read
		const Attribute<Bool>::Ptr mStoreNodalPowerInjection;

		/// Defines UID, name and logging level
		Transformer(String uid, String name,
			Logger::Level logLevel = Logger::Level::off, Bool withResistiveLosses = false);
		/// Defines name and logging level
		Transformer(String name, Logger::Level logLevel = Logger::Level::off)
			: Transformer(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name) override;

		// #### General ####
		/// Set transformer specific parameters (without rated power)
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);
		/// Set transformer specific parameters
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);
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
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector) override;
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector) override;
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Transformer& transformer) :
				Task(**transformer.mName + ".MnaPreStep"), mTransformer(transformer) {
					mTransformer.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mTransformer.mnaPreStep(time, timeStepCount); };
		private:
			Transformer& mTransformer;
		};


		class MnaPostStep : public Task {
		public:
			MnaPostStep(Transformer& transformer, Attribute<Matrix>::Ptr leftVector) :
				Task(**transformer.mName + ".MnaPostStep"), mTransformer(transformer), mLeftVector(leftVector) {
					mTransformer.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mTransformer.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			Transformer& mTransformer;
			Attribute<Matrix>::Ptr mLeftVector;
		};
    };
}
}
}
