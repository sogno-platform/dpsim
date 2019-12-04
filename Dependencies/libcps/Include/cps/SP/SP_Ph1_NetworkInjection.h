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
#include <cps/Solver/PFSolverInterfaceBus.h>
#include <cps/SP/SP_Ph1_PVNode.h>
#include <cps/SP/SP_Ph1_PQNode.h>
#include <cps/SP/SP_Ph1_VDNode.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Solver/DAEInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	///
    class externalGridInjection:
		public PowerComponent<Complex>,
		public SharedFactory<externalGridInjection>,
		public PFSolverInterfaceBus,
		public MNAInterface,
		public DAEInterface {

    private:

		// #### MNA ####
		///
		void updateVoltage(Real time);
		///
		Attribute<Complex>::Ptr mVoltageRef;
		///
		Attribute<Real>::Ptr mSrcFreq;

		// #### Powerflow section ####
		/// Voltage set point [V]
        Real mVoltageSetPoint;
		/// Apparent Power Injection [VA]
		Complex mPowerInjection;
		/// Active Power Injection [W]
		Real mActivePowerInjection;
		/// Reactive Power Injection [Var]
		Real mReactivePowerInjection;

		/// Voltage set point [pu]
		Real mVoltageSetPointPerUnit=1.0;

    public:
		// #### General ####
		/// Defines UID, name and logging level
		externalGridInjection(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		externalGridInjection(String name, Logger::Level logLevel = Logger::Level::off)
			: externalGridInjection(name, name, logLevel) { }
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		///
		void setSourceValue(Complex voltage);
		///
		void initialize(Matrix frequencies);
		/// 		
		PowerComponent<Complex>::Ptr clone(String name);

        // #### Powerflow section ####
		/// Set parameters relevant for powerflow solver
		void setParameters(Real vSetPointPerUnit);
        /// Modify powerflow bus type
		void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
		/// Update power injection
		void updatePowerInjection(Complex powerInj);
		
		// #### MNA Section ####
		/// Set parameters relevant for MNA solver
		void setParameters(Complex voltageRef, Real srcFreq = -1);
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(externalGridInjection& externalGridInjection) :
				Task(externalGridInjection.mName + ".MnaPreStep"), mExternalGridInjection(externalGridInjection) {
				mAttributeDependencies.push_back(externalGridInjection.attribute("V_ref"));
				mModifiedAttributes.push_back(mExternalGridInjection.attribute("right_vector"));
				mModifiedAttributes.push_back(mExternalGridInjection.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			externalGridInjection& mExternalGridInjection;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(externalGridInjection& externalGridInjection, Attribute<Matrix>::Ptr leftVector) :
				Task(externalGridInjection.mName + ".MnaPostStep"),
				mExternalGridInjection(externalGridInjection), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mExternalGridInjection.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			externalGridInjection& mExternalGridInjection;
			Attribute<Matrix>::Ptr mLeftVector;
		};


		// #### DAE Section ####
		/// Residual function for DAE Solver
		void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off);
		///Voltage Getter
		Complex daeInitialize();
	

};
}
}
}
