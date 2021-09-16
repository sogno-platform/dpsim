/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
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
#include <cps/SP/SP_Ph1_VoltageSource.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// \brief Network injection model
	///
	/// This model represents network injections by an ideal voltage source.
    class NetworkInjection:
		public SimPowerComp<Complex>,
		public SharedFactory<NetworkInjection>,
		public PFSolverInterfaceBus,
		public MNAInterface,
		public DAEInterface {

    private:
		// ### Electrical Subcomponents ###
		/// Voltage source
		std::shared_ptr<SP::Ph1::VoltageSource> mSubVoltageSource;

		// #### solver ####
		/// Vector to collect subcomponent right vector stamps
		std::vector<const Matrix*> mRightVectorStamps;

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
		/// Defines UID, name and logging level
		NetworkInjection(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		NetworkInjection(String name, Logger::Level logLevel = Logger::Level::off)
			: NetworkInjection(name, name, logLevel) { }
		///
		SimPowerComp<Complex>::Ptr clone(String name) override;

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) override;

        // #### Powerflow section ####
		/// Set parameters relevant for PF solver
		void setParameters(Real vSetPointPerUnit);
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
		void setParameters(Complex voltageRef, Real srcFreq = 0.0);
		/// Setter for reference signal of type frequency ramp
		void setParameters(Complex initialPhasor, Real freqStart, Real rocof, Real timeStart, Real duration, bool useAbsoluteCalc = true);
		/// Setter for reference signal of type cosine frequency modulation
		void setParameters(Complex initialPhasor, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency = 0.0, bool zigzag = false);
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector) override;
		/// Updates voltage across component
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(NetworkInjection& networkInjection) :
				Task(networkInjection.mName + ".MnaPreStep"), mNetworkInjection(networkInjection) {
					mNetworkInjection.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mNetworkInjection.mnaPreStep(time, timeStepCount); };

		private:
			NetworkInjection& mNetworkInjection;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(NetworkInjection& networkInjection, Attribute<Matrix>::Ptr leftVector) :
				Task(networkInjection.mName + ".MnaPostStep"), mNetworkInjection(networkInjection), mLeftVector(leftVector) {
				mNetworkInjection.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mNetworkInjection.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			NetworkInjection& mNetworkInjection;
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
