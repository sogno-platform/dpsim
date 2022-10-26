/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Signal/SineWaveGenerator.h>
#include <dpsim-models/Signal/SignalGenerator.h>
#include <dpsim-models/Signal/FrequencyRampGenerator.h>
#include <dpsim-models/Signal/CosineFMGenerator.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// \brief Ideal Voltage source model
	///
	/// This component uses a SignalGenerator instance to produce an output signal on mIntfVoltage.
	/// The signal generator can be configured to produce different signal shapes
	/// By default or when the setParameters(Complex voltageRef, Real srcFreq = 0.0) function is used to configure this VoltageSource,
	/// the SineWaveGenerator will be used. The frequency of the sine wave can be modified through the mSrcFreq attribute while the
	/// magnitude and phase of the wave are derived from the magnitude and phase of the mVoltageRef attribute. Refer to the formula
	/// in SineWaveGenerator.cpp for further details.
	/// When one of the other setParameters functions is used to configure this VoltageSource, the output signal will not react to changes in
	/// mVoltageRef or mSrcFreq. Instead, only the parameters given in the setParameters call are used to produce the signal.
	///
	/// This model uses modified nodal analysis to represent an ideal voltage source.
	/// For a voltage source between nodes j and k, a new variable
	/// (current across the voltage source) is added to the left side vector
	/// as unkown and it is taken into account for the equation of node j as
	/// positve and for the equation of node k as negative. Moreover
	/// a new equation ej - ek = V is added to the problem.
	class VoltageSource :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public DAEInterface,
		public SharedFactory<VoltageSource> {
	private:
	///
	void updateVoltage(Real time);
	///
	CPS::Signal::SignalGenerator::Ptr mSrcSig;
	public:
		const Attribute<Complex>::Ptr mVoltageRef;
		const Attribute<Real>::Ptr mSrcFreq;

		/// Defines UID, name, component parameters and logging level
		VoltageSource(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		/// Defines UID, name, component parameters and logging level
		VoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
			: VoltageSource(name, name, logLevel) { }
		/// Defines name, component parameters and logging level
		VoltageSource(String name,
			Complex voltage, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		///
		void setSourceValue(Complex voltage);
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

		// #### MNA Section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Returns current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);
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
			MnaPreStep(VoltageSource& voltageSource) :
				Task(**voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
					mVoltageSource.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
				}
				void execute(Real time, Int timeStepCount) { mVoltageSource.mnaPreStep(time, timeStepCount); };
		private:
			VoltageSource& mVoltageSource;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(VoltageSource& voltageSource, Attribute<Matrix>::Ptr leftVector) :
				Task(**voltageSource.mName + ".MnaPostStep"),
				mVoltageSource(voltageSource), mLeftVector(leftVector) {
					mVoltageSource.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount)  { mVoltageSource.mnaPostStep(time, timeStepCount, mLeftVector); };
		private:
			VoltageSource& mVoltageSource;
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
