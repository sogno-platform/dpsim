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
#include <cps/Base/Base_Ph3_PiLine.h>
#include <cps/EMT/EMT_Ph3_Resistor.h>
#include <cps/EMT/EMT_Ph3_Inductor.h>
#include <cps/EMT/EMT_Ph3_Capacitor.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// \brief PI-line dynamic phasor model
	///
	/// This model consists sub components to represent the
	/// RLC elements of a PI-line.
	class PiLine :
		public SimPowerComp<Real>,
		public MNAInterface,
		public Base::Ph3::PiLine,
		public SharedFactory<PiLine> {
	protected:
		/// Series Inductance submodel
		std::shared_ptr<Inductor> mSubSeriesInductor;
		/// Series Resistor submodel
		std::shared_ptr<Resistor> mSubSeriesResistor;
		/// Parallel Resistor submodel at Terminal 0
		std::shared_ptr<Resistor> mSubParallelResistor0;
		// Parallel Capacitor submodel at Terminal 0
		std::shared_ptr<Capacitor> mSubParallelCapacitor0;
		/// Parallel resistor submodel at Terminal 1
		std::shared_ptr<Resistor> mSubParallelResistor1;
		// Parallel capacitor submodel at Terminal 1
		std::shared_ptr<Capacitor> mSubParallelCapacitor1;
		/// solver
		std::vector<const Matrix*> mRightVectorStamps;
	public:
		/// Defines UID, name and logging level
		PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		PiLine(String name, Logger::Level logLevel = Logger::Level::off)
			: PiLine(name, name, logLevel) { }

		SimPowerComp<Real>::Ptr clone(String copySuffix);

		// #### General ####
		///
		void initialize(Matrix frequencies);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(PiLine& line) :
				Task(line.mName + ".MnaPreStep"), mLine(line) {
				mAttributeDependencies.push_back(line.mSubSeriesResistor->attribute("right_vector"));
				mAttributeDependencies.push_back(line.mSubSeriesInductor->attribute("right_vector"));
				if (line.mParallelCond(0,0) > 0) {
					mAttributeDependencies.push_back(line.mSubParallelResistor0->attribute("right_vector"));
					mAttributeDependencies.push_back(line.mSubParallelResistor1->attribute("right_vector"));
				}
				if (line.mParallelCap(0, 0) >= 0) {
					mAttributeDependencies.push_back(line.mSubParallelCapacitor0->attribute("right_vector"));
					mAttributeDependencies.push_back(line.mSubParallelCapacitor1->attribute("right_vector"));
				}

				mModifiedAttributes.push_back(line.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			PiLine& mLine;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(PiLine& line, Attribute<Matrix>::Ptr leftVector) :
				Task(line.mName + ".MnaPostStep"), mLine(line), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				mAttributeDependencies.push_back(line.mSubSeriesInductor->attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			PiLine& mLine;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		//MNAInterface::List mnaTearGroundComponents();
		/*void mnaTearInitialize(Real omega, Real timeStep);
		void mnaTearApplyMatrixStamp(Matrix& tearMatrix);
		void mnaTearApplyVoltageStamp(Matrix& voltageVector);
		void mnaTearPostStep(Matrix voltage, Matrix current);*/

	};
}
}
}
