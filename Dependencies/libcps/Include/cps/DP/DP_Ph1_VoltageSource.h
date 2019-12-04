/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <cps/PowerComponent.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Solver/DAEInterface.h>

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
		public PowerComponent<Complex>,
		public MNAInterface,
		public DAEInterface,
		public SharedFactory<VoltageSource> {
	private:
		///
		void updateVoltage(Real time);
		///
		Attribute<Complex>::Ptr mVoltageRef;
		///
		Attribute<Real>::Ptr mSrcFreq;
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
		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		///
		void setSourceValue(Complex voltage);
		///
		void initialize(Matrix frequencies);
		///
		void setParameters(Complex voltageRef, Real srcFreq = -1);

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

		class MnaPreStep : public Task {
		public:
			MnaPreStep(VoltageSource& voltageSource) :
				Task(voltageSource.mName + ".MnaPreStep"), mVoltageSource(voltageSource) {
				mAttributeDependencies.push_back(voltageSource.attribute("V_ref"));
				mModifiedAttributes.push_back(mVoltageSource.attribute("right_vector"));
				mModifiedAttributes.push_back(mVoltageSource.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			VoltageSource& mVoltageSource;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(VoltageSource& voltageSource, Attribute<Matrix>::Ptr leftVector) :
				Task(voltageSource.mName + ".MnaPostStep"),
				mVoltageSource(voltageSource), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mVoltageSource.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
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
			MnaPostStepHarm(VoltageSource& voltageSource, std::vector<Attribute<Matrix>::Ptr> leftVectors) :
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
