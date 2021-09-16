/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Base/Base_Ph1_VoltageSource.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
	/// Voltage source as Norton equivalent
	class VoltageSourceNorton :
		public SimPowerComp<Real>,
		public MNAInterface,
		public Base::Ph1::VoltageSource,
		public SharedFactory<VoltageSourceNorton> {
	protected:
		void updateState(Real time);

		/// Equivalent current source [A]
		Real mEquivCurrent;

		//  ### Real Voltage source parameters ###
		/// Resistance [ohm]
		Real mResistance;
		/// conductance [S]
		Real mConductance;
	public:
		/// Defines UID, name and logging level
		VoltageSourceNorton(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		VoltageSourceNorton(String name, Logger::Level logLevel = Logger::Level::off)
			: VoltageSourceNorton(name, name, logLevel) { }

		SimPowerComp<Real>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) { }
		///
		void setParameters(Complex voltage, Real srcFreq, Real resistance);
		///
		void setVoltageRef(Complex voltage) { mVoltageRef = voltage; }

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(VoltageSourceNorton& voltageSource) :
				Task(voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
				mAttributeDependencies.push_back(voltageSource.attribute("V_ref"));
				mModifiedAttributes.push_back(voltageSource.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			VoltageSourceNorton& mVoltageSource;
		};


		class MnaPostStep : public Task {
		public:
			MnaPostStep(VoltageSourceNorton& voltageSource, Attribute<Matrix>::Ptr leftVector) :
				Task(voltageSource.mName + ".MnaPostStep"), mVoltageSource(voltageSource),
				mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				mModifiedAttributes.push_back(voltageSource.attribute("i_intf"));
				mModifiedAttributes.push_back(voltageSource.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			VoltageSourceNorton& mVoltageSource;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
