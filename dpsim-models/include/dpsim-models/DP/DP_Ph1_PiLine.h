/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/MNATearInterface.h>
#include <dpsim-models/Base/Base_Ph1_PiLine.h>
#include <dpsim-models/DP/DP_Ph1_ResIndSeries.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>
#include <dpsim-models/DP/DP_Ph1_Capacitor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief PI-line dynamic phasor model
	///
	/// This model consists sub components to represent the
	/// RLC elements of a PI-line.
	class PiLine :
		public CompositePowerComp<Complex>,
		public MNATearInterface,
		public Base::Ph1::PiLine,
		public SharedFactory<PiLine> {
	protected:
		/// Series Resistor-Inductance submodel
		std::shared_ptr<ResIndSeries> mSubSeriesElement;
		/// Parallel Resistor submodel at Terminal 0
		std::shared_ptr<Resistor> mSubParallelResistor0;
		// Parallel Capacitor submodel at Terminal 0
		std::shared_ptr<Capacitor> mSubParallelCapacitor0;
		/// Parallel resistor submodel at Terminal 1
		std::shared_ptr<Resistor> mSubParallelResistor1;
		/// Parallel capacitor submodel at Terminal 1
		std::shared_ptr<Capacitor> mSubParallelCapacitor1;
		/// Right side vectors of subcomponents
		std::vector<const Matrix*> mRightVectorStamps;
	public:
		/// Defines UID, name and logging level
		PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		PiLine(String name, Logger::Level logLevel = Logger::Level::off)
			: PiLine(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String copySuffix);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Updates internal current variable of the component
		void mnaCompUpdateCurrent(const Matrix& leftVector);
		/// Updates internal voltage variable of the component
		void mnaCompUpdateVoltage(const Matrix& leftVector);
		/// MNA pre and post step operations
		void mnaParentPreStep(Real time, Int timeStepCount) override;
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// add MNA pre and post step dependencies
		void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		MNAInterface::List mnaTearGroundComponents();
		void mnaTearInitialize(Real omega, Real timeStep);
		void mnaTearApplyMatrixStamp(SparseMatrixRow& tearMatrix);
		void mnaTearApplyVoltageStamp(Matrix& voltageVector);
		void mnaTearPostStep(Complex voltage, Complex current);

	};
}
}
}
