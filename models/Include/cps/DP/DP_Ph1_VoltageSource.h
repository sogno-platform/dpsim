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
#include <cps/Solver/DAEInterface.h>
#include <cps/Signal/SineWaveGenerator.h>
#include <cps/Signal/SignalGenerator.h>
#include <cps/Signal/FrequencyRampGenerator.h>
#include <cps/Signal/CosineFMGenerator.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief Ideal Voltage source model
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
		///
		//void setSourceSignal(CPS::Signal::SignalGenerator::Ptr srcSig);
		///
		void setParameters(Complex voltageRef, Real srcFreq = 0.0);
		/// Setter for reference signal of type frequency ramp
		void setParameters(Complex initialPhasor, Real freqStart, Real rocof, Real timeStart, Real duration, bool useAbsoluteCalc = true);
		/// Setter for reference signal of type cosine frequency modulation
		void setParameters(Complex initialPhasor, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency = 0.0, bool zigzag = false);

		// #### MNA Section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaApplyRightSideVectorStampHarm(Matrix& rightVector);
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

		class MnaPreStepHarm : public CPS::Task {
		public:
			MnaPreStepHarm(VoltageSource& voltageSource) :
				Task(voltageSource.mName + ".MnaPreStepHarm"),
				mVoltageSource(voltageSource) {
				mAttributeDependencies.push_back(voltageSource.attribute("V_ref"));
				mModifiedAttributes.push_back(mVoltageSource.attribute("right_vector"));
				mModifiedAttributes.push_back(mVoltageSource.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			VoltageSource& mVoltageSource;
		};

		class MnaPostStepHarm : public CPS::Task {
		public:
			MnaPostStepHarm(VoltageSource& voltageSource, const std::vector<Attribute<Matrix>::Ptr> &leftVectors) :
				Task(voltageSource.mName + ".MnaPostStepHarm"),
				mVoltageSource(voltageSource), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mVoltageSource.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			VoltageSource& mVoltageSource;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
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
