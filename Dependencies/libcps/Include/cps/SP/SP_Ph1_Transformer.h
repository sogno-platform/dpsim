/*
 * Copyright 2019 The DPsim Authors. All Rights Reserved.
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
#include <cps/PowerComponent.h>
#include <cps/Solver/PFSolverInterfaceBranch.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/SP/SP_Ph1_Resistor.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Shunt.h>
#include <cps/Base/Base_Ph1_Transformer.h>

namespace CPS {

namespace SP { namespace Ph1 {
	/// Transformer that includes an inductance and resistance
	class Transformer :
		public PowerComponent<Complex>,
		public SharedFactory<Transformer>,
		public PFSolverInterfaceBranch,
		public MNAInterface,
		public Base::Ph1::Transformer {

	private:
		/// Internal inductor to model losses
		std::shared_ptr<SP::Ph1::Inductor> mSubInductor;
		/// Internal parallel resistance as snubber
		std::shared_ptr<SP::Ph1::Resistor> mSubSnubResistor;
		///
		std::shared_ptr<SP::Ph1::Resistor> mSubResistor;

		/// Rated Apparent Power [VA]
		Real mRatedPower = 0;
		/// Transformer ratio complex
		Complex mRatio;
        /// Transformer ratio magnitude
		Real mRatioAbs = 1;
        /// Transformer ratio pase [deg]
		Real mRatioPhase = 0;
		/// Nominal voltage of primary side
		Real mNominalVoltageEnd1;
		/// Nominal voltage of secondary side
		Real mNominalVoltageEnd2;
		/// Nominal omega
		Real mNominalOmega;

		/// Voltage [V]
		Real mSvVoltage;
		/// Conductance [S]
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
		/// base voltage [V]
		Real mBaseVoltage;
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

		// #### Power flow results ####
		/// branch Current flow [A]
		Eigen::Matrix<CPS::Complex, 2, 1, Eigen::DontAlign> mCurrent;
		/// branch active powerflow [W], coef(0) has data from node 0, coef(1) from node 1.
		Eigen::Matrix<CPS::Real, 2, 1, Eigen::DontAlign> mActivePowerBranch;
		/// branch reactive powerflow [Var]
		Eigen::Matrix<CPS::Real, 2, 1, Eigen::DontAlign> mReactivePowerBranch;
		/// whether the total power injection of its from node is stored in this line
		Bool mStoreNodalPowerInjection = false;
		/// nodal active power injection
		Real mActivePowerInjection;
		/// nodal reactive power injection
		Real mReactivePowerInjection;

	public:
		/// Defines UID, name and logging level
		Transformer(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		Transformer(String name, Logger::Level logLevel = Logger::Level::off)
			: Transformer(name, name, logLevel) { }

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Set transformer specific parameters
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance, Real omega);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

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
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Transformer& transformer, Attribute<Matrix>::Ptr leftVector) :
				Task(transformer.mName + ".MnaPostStep"), mTransformer(transformer), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(transformer.mSubInductor->attribute("i_intf"));
				mAttributeDependencies.push_back(leftVector);
				mModifiedAttributes.push_back(transformer.attribute("i_intf"));
				mModifiedAttributes.push_back(transformer.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Transformer& mTransformer;
			Attribute<Matrix>::Ptr mLeftVector;
		};
    };
}
}
}
