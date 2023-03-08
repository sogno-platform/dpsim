/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/PFSolverInterfaceBus.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/SP/SP_Ph1_VoltageSource.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// \brief Network injection model
	///
	/// This model represents network injections by an ideal voltage source.
	/// The voltage source can be configured to use different types of SignalGenerators using the various setParameters functions
	/// When the SineWaveGenerator is configured via the void setParameters(Complex voltageRef, Real srcFreq = 0.0) function,
	/// the frequency, magnitude and phase of the sine wave can be modified through the mVoltageRef and mSrcFreq attributes.
	/// See SP_Ph1_VoltageSource.h for more details.
    class NetworkInjection:
		public CompositePowerComp<Complex>,
		public SharedFactory<NetworkInjection>,
		public PFSolverInterfaceBus,
		public DAEInterface {

    private:
		// ### Electrical Subcomponents ###
		/// Voltage source
		std::shared_ptr<SP::Ph1::VoltageSource> mSubVoltageSource;

		// #### solver ####
		/// Vector to collect subcomponent right vector stamps
		std::vector<const Matrix*> mRightVectorStamps;

		// #### Powerflow section ####
		/// Base voltage [V]
		const Attribute<Real>::Ptr mBaseVoltage;

    public:
		const Attribute<Complex>::Ptr mVoltageRef;
		const Attribute<Real>::Ptr mSrcFreq;

		// #### Powerflow section ####
		/// Voltage set point [V]
        const Attribute<Real>::Ptr mVoltageSetPoint;
		/// Voltage set point [pu]
		const Attribute<Real>::Ptr mVoltageSetPointPerUnit;
		/// Active Power Injection [W]
		const Attribute<Real>::Ptr mActivePowerInjection;
		/// Reactive Power Injection [Var]
		const Attribute<Real>::Ptr mReactivePowerInjection;

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
		/// Setter for reference voltage and frequency with a sine wave generator
		/// This will initialize the values of mVoltageRef and mSrcFreq to match the given parameters
		/// However, the attributes can be modified during the simulation to dynamically change the magnitude, frequency, and phase of the sine wave.
		void setParameters(Complex voltageRef, Real srcFreq = 0.0);
		/// Setter for reference signal of type frequency ramp
		/// This will create a FrequencyRampGenerator which will not react to external changes to mVoltageRef or mSrcFreq!
		void setParameters(Complex initialPhasor, Real freqStart, Real rocof, Real timeStart, Real duration, bool smoothRamp = true);
		/// Setter for reference signal of type cosine frequency modulation
		/// This will create a CosineFMGenerator which will not react to external changes to mVoltageRef or mSrcFreq!
		void setParameters(Complex initialPhasor, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency = 0.0, bool zigzag = false);

		/// Stamps right side (source) vector
		void mnaParentApplyRightSideVectorStamp(Matrix& rightVector) override;
		/// Returns current through the component
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// Updates voltage across component
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// MNA pre step operations
		void mnaParentPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		// #### DAE Section ####
		/// Residual function for DAE Solver
		void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) override;
		///Voltage Getter
		Complex daeInitialize() override;


};
}
}
}
