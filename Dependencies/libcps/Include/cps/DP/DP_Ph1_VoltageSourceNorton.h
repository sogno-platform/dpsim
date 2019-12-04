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
#include <cps/Base/Base_Ph1_VoltageSource.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief Resistive dynamic phasor voltage source
	///
	/// The real voltage source is a voltage source in series with a resistance,
	/// which is transformed to a current source with
	/// a parallel resistance using the Norton equivalent.
	class VoltageSourceNorton :
		public PowerComponent<Complex>,
		public Base::Ph1::VoltageSource,
		public MNAInterface,
		public SharedFactory<VoltageSourceNorton> {
	protected:
		/// Equivalent current source [A]
		Complex mEquivCurrent;

		//  ### Real Voltage source parameters ###
		/// Resistance [ohm]
		Real mResistance;
		/// conductance [S]
		Real mConductance;

		/// Helper function for PreStep
		void updateState(Real time);
	public:
		/// Defines UID, name and logging level
		VoltageSourceNorton(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		VoltageSourceNorton(String name, Logger::Level logLevel = Logger::Level::off)
			: VoltageSourceNorton(name, name, logLevel) { }

		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency) { }
		///
		void setVoltageRef(Complex voltage) { mVoltageRef = voltage; }
		///
		using Base::Ph1::VoltageSource::setParameters;
		///
		void setParameters(Complex voltage, Real srcFreq = -1, Real resistance = 1e9);

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
