/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/MNATearInterface.h>
#include <dpsim-models/Solver/PFSolverInterfaceBranch.h>
#include <dpsim-models/Base/Base_Ph1_PiLine.h>
#include <dpsim-models/SP/SP_Ph1_ResIndSeries.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>
#include <dpsim-models/SP/SP_Ph1_Capacitor.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// \brief PI-line static phasor model
	///
	/// For MNA this model consists sub components to represent the
	/// RLC elements of a PI-line.
	class PiLine :
	 public CompositePowerComp<Complex>,
	 public Base::Ph1::PiLine,
	 public MNATearInterface,
	 public SharedFactory<PiLine>,
	 public PFSolverInterfaceBranch {
	public:
		///base voltage [V]
		const Attribute<Real>::Ptr mBaseVoltage;

		// #### Power flow results ####
		/// branch Current flow [A], coef(0) has data from node 0, coef(1) from node 1.
		const Attribute<MatrixComp>::Ptr mCurrent;

		/// branch active powerflow [W], coef(0) has data from node 0, coef(1) from node 1.
		const Attribute<Matrix>::Ptr mActivePowerBranch;

		/// branch reactive powerflow [Var], coef(0) has data from node 0, coef(1) from node 1.
		const Attribute<Matrix>::Ptr mReactivePowerBranch;

	protected:
		/// CHECK: Which of these really need to be member variables?
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
		/// Series Resistor-Inductance submodel
		std::shared_ptr<ResIndSeries> mSubSeriesElement;
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
		/// nodal active power injection
		const Attribute<Real>::Ptr mActivePowerInjection;
		/// nodal reactive power injection
		const Attribute<Real>::Ptr mReactivePowerInjection;

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
		/// Updates internal current variable of the component
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// Updates internal voltage variable of the component
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// MNA post-step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// add MNA post-step dependencies
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		MNAInterface::List mnaTearGroundComponents() override;
		void mnaTearInitialize(Real omega, Real timeStep) override;
		void mnaTearApplyMatrixStamp(SparseMatrixRow& tearMatrix) override;
		void mnaTearApplyVoltageStamp(Matrix& voltageVector) override;
		void mnaTearPostStep(Complex voltage, Complex current) override;
	};
}
}
}
