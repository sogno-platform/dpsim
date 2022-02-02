/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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
			/// \brief Ideal current source model
			///
			/// This model uses modified nodal analysis to represent an ideal current source.
			/// This involves the stamping of the current to the right side vector.
			class CurrentSource :
				public MNAInterface,
				public SimPowerComp<Real>,
				public SharedFactory<CurrentSource> {
			private:
				/// 
				CPS::Signal::SignalGenerator::Ptr mSrcSig;
			protected:
				// Updates current according to reference phasor and frequency
				void updateCurrent(Real time);
			public:
				const Attribute<MatrixComp>::Ptr mCurrentRef;
				const Attribute<Real>::Ptr mSrcFreq;
				const Attribute<Complex>::Ptr mSigOut;

				/// Defines UID, name and logging level
				CurrentSource(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				CurrentSource(String name, Logger::Level logLevel = Logger::Level::off)
					: CurrentSource(name, name, logLevel) { }

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
				/// Stamps right side (source) vector
				void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				/// Returns voltage through the component
				void mnaUpdateVoltage(const Matrix& leftVector);
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
					MnaPreStep(CurrentSource& currentSource) :
						Task(**currentSource.mName + ".MnaPreStep"), mCurrentSource(currentSource) {
							mCurrentSource.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
						}
						void execute(Real time, Int timeStepCount) { mCurrentSource.mnaPreStep(time, timeStepCount); };
				private:
					CurrentSource& mCurrentSource;
				};

				class MnaPostStep : public Task {
				public:
					MnaPostStep(CurrentSource& currentSource, Attribute<Matrix>::Ptr leftVector) :
						Task(**currentSource.mName + ".MnaPostStep"),			
						mCurrentSource(currentSource), mLeftVector(leftVector) {
							mCurrentSource.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
					}
					void execute(Real time, Int timeStepCount)  { mCurrentSource.mnaPostStep(time, timeStepCount, mLeftVector); };
				private:
					CurrentSource& mCurrentSource;
					Attribute<Matrix>::Ptr mLeftVector;
				};
			};
		}
	}
}
