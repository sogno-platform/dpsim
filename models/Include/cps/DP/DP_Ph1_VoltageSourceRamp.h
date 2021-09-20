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
#include <cps/DP/DP_Ph1_VoltageSource.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	class VoltageSourceRamp :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<VoltageSourceRamp> {
	protected:
		///
		Complex mVoltageRef;
		///
		Complex mAddVoltage;
		///
		Real mSrcFreq;
		///
		Real mSwitchTime;
		///
		Real mAddSrcFreq;
		///
		Real mRampTime;
		///
		std::shared_ptr<VoltageSource> mSubVoltageSource;

		void updateState(Real time);
	public:
		/// Defines UID, name and logging level
		VoltageSourceRamp(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		VoltageSourceRamp(String name, Logger::Level logLevel = Logger::Level::off)
			: VoltageSourceRamp(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		///
		void setParameters(Complex voltage, Complex addVoltage,
			Real srcFreq, Real addSrcFreq, Real switchTime, Real rampTime);
		///
		void initialize(Matrix frequencies);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);


		class MnaPreStep : public Task {
		public:
			MnaPreStep(VoltageSourceRamp& voltageSource) :
				Task(voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
				// rampTime etc. aren't attributes (yet), so doesn't really depend on anything
				mModifiedAttributes.push_back(voltageSource.mSubVoltageSource->attribute("V_ref"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			VoltageSourceRamp& mVoltageSource;
		};
	};
}
}
}
