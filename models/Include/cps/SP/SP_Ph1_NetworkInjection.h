/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/PFSolverInterfaceBus.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Solver/DAEInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	///
    class externalGridInjection:
		public SimPowerComp<Complex>,
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

		/// Base voltage [V]
		Real mBaseVoltage;

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
		void initializeFromPowerflow(Real frequency) override;
		///
		void setSourceValue(Complex voltage);
		///
		void setParameters(Real vSetPointPerUnit);
		///
		SimPowerComp<Complex>::Ptr clone(String name) override;

        // #### Powerflow section ####
		/// Set base voltage
		void setBaseVoltage(Real baseVoltage);
		/// Calculates component's parameters in specified per-unit system
		void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
        /// Modify powerflow bus type
		void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
		/// Update power injection
		void updatePowerInjection(Complex powerInj);

		// #### MNA Section ####
		/// Set parameters relevant for MNA solver
		void setParameters(Complex voltageRef, Real srcFreq = -1);
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector) override;

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
		void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) override;
		///Voltage Getter
		Complex daeInitialize() override;


};
}
}
}
