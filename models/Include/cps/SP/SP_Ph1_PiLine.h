/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNATearInterface.h>
#include <cps/Solver/PFSolverInterfaceBranch.h>
#include <cps/Base/Base_Ph1_PiLine.h>
#include <cps/SP/SP_Ph1_Resistor.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Capacitor.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// \brief PI-line static phasor model
	///
	/// For MNA this model consists sub components to represent the
	/// RLC elements of a PI-line.
	class PiLine :
	 public SimPowerComp<Complex>,
	 public MNATearInterface,
	 public SharedFactory<PiLine>,
	 public Base::Ph1::PiLine,
	 public PFSolverInterfaceBranch {
	protected:
		///base voltage [V]
		Real mBaseVoltage;
		///base current [A]
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
		///Conductance of the line in [pu]
		Real mParallelCondPerUnit;

		// #### Admittance matrix stamp ####
		MatrixComp mY_element;

		// #### Power flow results ####
		/// branch Current flow [A]
		Eigen::Matrix<CPS::Complex, 2, 1,Eigen::DontAlign> mCurrent;
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
		/// Series Inductance submodel
		std::shared_ptr<Inductor> mSubSeriesInductor;
		/// Series Resistor submodel
		std::shared_ptr<Resistor> mSubSeriesResistor;
		/// Parallel Resistor submodel at Terminal 0
		std::shared_ptr<Resistor> mSubParallelResistor0;
		// Parallel Capacitor submodel at Terminal 0
		std::shared_ptr<Capacitor> mSubParallelCapacitor0;
		/// Parallel resistor submodel at Terminal 1
		std::shared_ptr<Resistor> mSubParallelResistor1;
		/// Parallel capacitor submodel at Terminal 1
		std::shared_ptr<Capacitor> mSubParallelCapacitor1;
		/// Right side vectors of subcomponents
		std::vector<const Matrix*> mRightVectorStamps;
	public:
		// #### General ####
		/// Defines UID, name and logging level
		PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		PiLine(String name, Logger::Level logLevel = Logger::Level::off)
			: PiLine(name, name, logLevel) { }
		///
		SimPowerComp<Complex>::Ptr clone(String copySuffix) override;
		///
		void setParameters(Real resistance, Real inductance, Real capacitance = -1, Real conductance = -1);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) override;

		// #### Powerflow section ####
		/// Set base voltage
		void setBaseVoltage(Real baseVoltage);
		/// Calculates component's parameters in specified per-unit system
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

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector) override;
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector) override;
		/// MNA post-step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// add MNA post-step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(PiLine& line, Attribute<Matrix>::Ptr leftVector) :
				Task(line.mName + ".MnaPostStep"), mLine(line), mLeftVector(leftVector) {
					mLine.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mLine.mnaPostStep(time, timeStepCount, mLeftVector); };
		private:
			PiLine& mLine;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		MNAInterface::List mnaTearGroundComponents() override;
		void mnaTearInitialize(Real omega, Real timeStep) override;
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix) override;
		void mnaTearApplyVoltageStamp(Matrix& voltageVector) override;
		void mnaTearPostStep(Complex voltage, Complex current) override;
	};
}
}
}
