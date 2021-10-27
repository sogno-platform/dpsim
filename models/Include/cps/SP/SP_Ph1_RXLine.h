/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include "cps/Solver/PFSolverInterfaceBranch.h"
#include <cps/Base/Base_Ph1_PiLine.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Resistor.h>

namespace CPS {
namespace SP {
namespace Ph1 {

	class RXLine :
		public SimPowerComp<Complex>,
		public SharedFactory<RXLine>,
		public PFSolverInterfaceBranch,
		public Base::Ph1::PiLine,
		public MNAInterface{
	protected:
		///Capacitance of the line in [F]
		Real mCapacitance;
		///Inductance of the line in [H]
		Real mInductance;

		///base voltage [V]
		Real mBaseVoltage;
		///base current [V]
		Real mBaseCurrent;
		///base apparent power [VA]
		Real mBaseApparentPower;
		///base omega [1/s]
		Real mBaseOmega;
		///base impedance [Ohm]
		Real mBaseImpedance;
		///base admittance [S]
		Real mBaseAdmittance;
		///base inductance [H]
		Real mBaseInductance;
		///base capacitance [F]
		Real mBaseCapacitance;

		///resistance in [pu]
		Real mSeriesResPerUnit;
		///Capacitance of the line in [pu]
		Real mParallelCapPerUnit;
		///Inductance of the line in [pu]
		Real mSeriesIndPerUnit;

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


		/// Inductance submodel
		std::shared_ptr<Inductor> mSubInductor;
		/// Resistor submodel
		std::shared_ptr<Resistor> mSubResistor;
		/// Inductor end to ground resistor to facilitate initialization
		std::shared_ptr<Resistor> mInitialResistor;

	public:
		// #### constructors ####
		// power flow

		/// Defines UID, name, base voltage, component parameters and logging level
		RXLine(String uid, String name, Real baseVoltage,
			Real resistance, Real inductance,
			Logger::Level logLevel = Logger::Level::off);
		// MNA
		/// Defines UID, name, logging level
		RXLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RXLine(String name, Logger::Level logLevel = Logger::Level::off)
			: RXLine(name, name, logLevel) { }

		// #### General ####
		/// Specify per-unit system by base voltage, base apparent power and omega (rms value for voltage expected)
		void setPerUnitSystem(Real baseApparentPower, Real baseOmega);
		/// Transform component parameters to the specified per-unit system
		void transformParametersToPerUnitSystem();

		// #### Powerflow section ####
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


		SimPowerComp<Complex>::Ptr clone(String name) override;

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) override;

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Stamps system matrix
		void mnaApplyInitialSystemMatrixStamp(Matrix& systemMatrix);
		void mnaUpdateVoltage(const Matrix& leftVector) override;
		void mnaUpdateCurrent(const Matrix& leftVector) override;

		class MnaPreStep : public Task {
		public:
			MnaPreStep(RXLine& line) :
				Task(line.mName + ".MnaPreStep"), mLine(line) {
				mAttributeDependencies.push_back(line.mSubResistor->attribute("right_vector"));
				mAttributeDependencies.push_back(line.mSubInductor->attribute("right_vector"));
				mModifiedAttributes.push_back(line.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			RXLine& mLine;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(RXLine& line, Attribute<Matrix>::Ptr leftVector) :
				Task(line.mName + ".MnaPostStep"), mLine(line), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				mAttributeDependencies.push_back(line.mSubInductor->attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			RXLine& mLine;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
