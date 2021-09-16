/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Base/Base_Ph1_Inductor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief resistor inductor series element
	class ResIndSeries :
		public MNATearInterface,
		public SimPowerComp<Complex>,
		public SharedFactory<ResIndSeries> {
	protected:
		/// Inductance [H]
		Real mInductance;
		///Resistance [ohm]
		Real mResistance;
		///Conductance [S]
		Real mConductance;
		/// DC equivalent current source for harmonics [A]
		MatrixComp mEquivCurrent;
		/// Equivalent conductance for harmonics [S]
		MatrixComp mEquivCond;
		/// Coefficient in front of previous current value for harmonics
		MatrixComp mPrevCurrFac;
	public:
		/// Defines UID, name and log level
		ResIndSeries(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		ResIndSeries(String name, Logger::Level logLevel = Logger::Level::off)
			: Inductor(name, name, logLevel) { }

		// #### General ####
		/// Sets model specific parameters
		void setParameters(Real resistance, Real inductance) {
			mResistance = resistance;
			mInductance = inductance;
		}
		/// Return new instance with the same parameters
		SimPowerComp<Complex>::Ptr clone(String name);
		/// Initializes state variables considering the number of frequencies
		void initialize(Matrix frequencies);
		/// Initializes states from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface voltage from MNA system results
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface current from MNA system results
		void mnaUpdateCurrent();

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Inductor& ResIndSeries) :
				Task(inductor.mName + ".MnaPreStep"), mResIndSeries(resIndSeries) {
				// actually depends on L, but then we'd have to modify the system matrix anyway
				mModifiedAttributes.push_back(mResIndSeries.attribute("right_vector"));
				mPrevStepDependencies.push_back(mResIndSeries.attribute("v_intf"));
				mPrevStepDependencies.push_back(mResIndSeries.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			ResIndSeries& mResIndSeries;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(ResIndSeries& inductor, Attribute<Matrix>::Ptr leftVector) :
				Task(inductor.mName + ".MnaPostStep"),
				mResIndSeries(resIndSeries), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mResIndSeries.attribute("v_intf"));
				mModifiedAttributes.push_back(mResIndSeries.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			ResIndSeries& mResIndSeries;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
