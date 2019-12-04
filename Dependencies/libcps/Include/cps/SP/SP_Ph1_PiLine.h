/**
 * @file
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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
#include <cps/Solver/MNATearInterface.h>
#include <cps/Solver/PFSolverInterfaceBranch.h>
#include <cps/Base/Base_Ph1_PiLine.h>
#include <cps/SP/SP_Ph1_Resistor.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Capacitor.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	class PiLine :
	 public PowerComponent<Complex>,
	 public MNATearInterface,
	 public SharedFactory<PiLine>,
	 public Base::Ph1::PiLine,
	 public PFSolverInterfaceBranch {
	protected:
		/// Nominal omega
		Real mNominalOmega;

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
		// Parallel capacitor submodel at Terminal 1
		std::shared_ptr<Capacitor> mSubParallelCapacitor1;
		///
		std::vector<const Matrix*> mRightVectorStamps;
	public:
		// #### General ####
		/// Defines UID, name and logging level
		PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		PiLine(String name, Logger::Level logLevel = Logger::Level::off)
			: PiLine(name, name, logLevel) { }
		///
		PowerComponent<Complex>::Ptr clone(String copySuffix);
		///
		void setParameters(Real resistance, Real inductance, Real capacitance = -1, Real conductance = -1, Real omega = -1);
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

		// #### MNA section ####
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
			MnaPostStep(PiLine& line, Attribute<Matrix>::Ptr leftVector) :
				Task(line.mName + ".MnaPostStep"), mLine(line), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				mAttributeDependencies.push_back(line.mSubSeriesInductor->attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			PiLine& mLine;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		MNAInterface::List mnaTearGroundComponents();
		void mnaTearInitialize(Real omega, Real timeStep);
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);
		void mnaTearApplyVoltageStamp(Matrix& voltageVector);
		void mnaTearPostStep(Complex voltage, Complex current);
	};
}
}
}
