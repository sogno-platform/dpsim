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
#include <cps/Solver/MNATearInterface.h>
#include <cps/Solver/DAEInterface.h>
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief Dynamic phasor resistor model
	class Resistor :
		public Base::Ph1::Resistor,
		public MNATearInterface,
		public DAEInterface,
		public PowerComponent<Complex>,
		public SharedFactory<Resistor> {
	public:
		/// Defines UID, name and logging level
		Resistor(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		/// Defines name and logging level
		Resistor(String name, Logger::Level logLevel = Logger::Level::off)
			: Resistor(name, name, logLevel) { }

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initialize components with correct network frequencies
		void initialize(Matrix frequencies);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps system matrix considering the frequency index
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
		void mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);
		void mnaUpdateCurrentHarm();

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Resistor& resistor, Attribute<Matrix>::Ptr leftSideVector) :
				Task(resistor.mName + ".MnaPostStep"),
				mResistor(resistor), mLeftVector(leftSideVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
				mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Resistor& mResistor;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		class MnaPostStepHarm : public Task {
		public:
			MnaPostStepHarm(Resistor& resistor, std::vector<Attribute<Matrix>::Ptr> leftVectors) :
				Task(resistor.mName + ".MnaPostStepHarm"),
				mResistor(resistor), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
				mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Resistor& mResistor;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};

		// #### MNA Tear Section ####
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);

		// #### DAE Section ####
		///Residual Function for DAE Solver
		void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off);
		///Voltage Getter
		Complex daeInitialize();
	};
}
}
}
