/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Signal/SignalGenerator.h>
#include <cps/Signal/SineWaveGenerator.h>
#include <cps/Signal/FrequencyRampGenerator.h>
#include <cps/Signal/CosineFMGenerator.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// \brief Ideal Voltage source model
			///
			/// This model uses modified nodal analysis to represent an ideal voltage source.
			/// For a voltage source between nodes j and k, a new variable (current across the voltage source)
			/// is added to the left side vector
			/// as unkown and it is taken into account for the equation of node j as positve and for the equation
			/// of node k as negative. Moreover
			/// a new equation ej - ek = V is added to the problem.
			class VoltageSource :
				public MNAInterface,
				public SimPowerComp<Real>,
				public SharedFactory<VoltageSource> {
			private:
				///
				CPS::Signal::SignalGenerator::Ptr mSrcSig;
			protected:
				// Updates voltage according to reference phasor and frequency
				void updateVoltage(Real time);
			public:
				/// Defines UID, name and logging level
				VoltageSource(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				VoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
					: VoltageSource(name, name, logLevel) { }

				SimPowerComp<Real>::Ptr clone(String name);
				// #### General ####
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency);
				/// Setter for reference voltage
				void setParameters(MatrixComp voltageRef, Real srcFreq = 50.0);
				/// Setter for reference signal of type frequency ramp
				void setParameters(MatrixComp voltageRef, Real freqStart, Real rocof, Real timeStart, Real duration, bool useAbsoluteCalc = true);
				/// Setter for reference signal of type cosine frequency modulation
				void setParameters(MatrixComp voltageRef, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency = 50.0, bool zigzag = false);

				// #### MNA section ####
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
						Task(voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
							mVoltageSource.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
						}
						void execute(Real time, Int timeStepCount) { mVoltageSource.mnaPreStep(time, timeStepCount); };
				private:
					VoltageSource& mVoltageSource;
				};

				class MnaPostStep : public Task {
				public:
					MnaPostStep(VoltageSource& voltageSource, Attribute<Matrix>::Ptr leftVector) :
						Task(voltageSource.mName + ".MnaPostStep"),
						mVoltageSource(voltageSource), mLeftVector(leftVector) {
							mVoltageSource.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
					}
					void execute(Real time, Int timeStepCount)  { mVoltageSource.mnaPostStep(time, timeStepCount, mLeftVector); };
				private:
					VoltageSource& mVoltageSource;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
