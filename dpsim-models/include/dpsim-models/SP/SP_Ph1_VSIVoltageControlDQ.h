/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/SP/SP_Ph1_ResIndSeries.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>
#include <dpsim-models/SP/SP_Ph1_Capacitor.h>
#include <dpsim-models/SP/SP_Ph1_VoltageSource.h>
#include <dpsim-models/SP/SP_Ph1_Transformer.h>
#include <dpsim-models/Base/Base_VSIVoltageSourceInverterDQ.h>
#include <dpsim-models/Signal/VoltageControllerVSI.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	class VSIVoltageControlDQ :
		public CompositePowerComp<Complex>,
		public Base::VSIVoltageSourceInverterDQ,
		public SharedFactory<VSIVoltageControlDQ> {
	protected:
	
		// ### Electrical Subcomponents ###
		/// Controlled voltage source
		std::shared_ptr<SP::Ph1::VoltageSource> mSubCtrledVoltageSource;
		/// RL Element as part of LC filter
		std::shared_ptr<SP::Ph1::ResIndSeries> mSubFilterRL;
		/// Capacitor Cf as part of LC filter
		std::shared_ptr<SP::Ph1::Capacitor> mSubCapacitorF;
		/// Resistor Rc as part of LC filter
		std::shared_ptr<SP::Ph1::Resistor> mSubResistorC;
		/// Optional connection transformer
		std::shared_ptr<SP::Ph1::Transformer> mConnectionTransformer;

	public:
		// ### General Parameters ###

		/// Defines name amd logging level
		VSIVoltageControlDQ(String name, Logger::Level logLevel = Logger::Level::off)
			: VSIVoltageControlDQ(name, name, logLevel) {}
		/// Defines UID, name, logging level and connection trafo existence
		VSIVoltageControlDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaParentInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Updates current through the component
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// Updates voltage across component
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;
		/// MNA pre step operations
		void mnaParentPreStep(Real time, Int timeStepCount) override;
		/// MNA post step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
		/// Add MNA pre step dependencies
		void mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		/// Add MNA post step dependencies
		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

	private:
		///
		void createSubComponents() final;
		///
		void connectSubComponents();
	};
}
}
}
