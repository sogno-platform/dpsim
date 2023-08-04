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
#include <dpsim-models/Definitions.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Capacitor.h>
#include <dpsim-models/DP/DP_Ph1_VoltageSource.h>
#include <dpsim-models/DP/DP_Ph1_Transformer.h>
#include <dpsim-models/Base/Base_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/Signal/VCO.h>
#include <dpsim-models/Signal/VoltageControllerVSI.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	class VSIVoltageControlDQ :
		public CompositePowerComp<Complex>,
		public Base::AvVoltageSourceInverterDQ,
		public SharedFactory<VSIVoltageControlDQ> {
	protected:

		// ### General Parameters ###
		/// Nominal system angle
		/// CHECK: Should this be an Attribute?
		Real mThetaN = 0;
		/// Nominal voltage
		/// CHECK: Should this be an Attribute?
		Real mVnom;
		/// Simulation step
		Real mTimeStep;


		// ### Control Subcomponents ###
		/// VCO
		std::shared_ptr<Signal::VCO> mVCO;
		/// Power Controller
		std::shared_ptr<Signal::VoltageControllerVSI> mVoltageControllerVSI;

		// ### Electrical Subcomponents ###
		/// Controlled voltage source
		std::shared_ptr<DP::Ph1::VoltageSource> mSubCtrledVoltageSource;
		/// Resistor Rf as part of LCL filter
		std::shared_ptr<DP::Ph1::Resistor> mSubResistorF;
		/// Capacitor Cf as part of LCL filter
		std::shared_ptr<DP::Ph1::Capacitor> mSubCapacitorF;
		/// Inductor Lf as part of LCL filter
		std::shared_ptr<DP::Ph1::Inductor> mSubInductorF;
		/// Resistor Rc as part of LCL filter
		std::shared_ptr<DP::Ph1::Resistor> mSubResistorC;
		/// Optional connection transformer
		std::shared_ptr<DP::Ph1::Transformer> mConnectionTransformer;

		/// Flag for connection transformer usage
		Bool mWithConnectionTransformer=false;
		/// Flag for controller usage
		Bool mWithControl=true;

	public:
		// ### General Parameters ###

		/// Nominal frequency
		const Attribute<Real>::Ptr mOmegaN;
		/// Voltage d reference
		const Attribute<Real>::Ptr mVdRef;
		/// Voltage q reference
		const Attribute<Real>::Ptr mVqRef;

		// ### Inverter Interfacing Variables ###
		// Control inputs
		/// Measured voltage d-axis in local reference frame
		const Attribute<Real>::Ptr mVcd;
		/// Measured voltage q-axis in local reference frame
		const Attribute<Real>::Ptr mVcq;
		/// Measured current d-axis in local reference frame
		const Attribute<Real>::Ptr mIrcd;
		/// Measured current q-axis in local reference frame
		const Attribute<Real>::Ptr mIrcq;
		const Attribute<Real>::Ptr mElecActivePower;
		const Attribute<Real>::Ptr mElecPassivePower;
		// Control outputs
		/// Voltage as control output after transformation interface
		const Attribute<MatrixComp>::Ptr mVsref;

		// Sub voltage source
		const Attribute<MatrixComp>::Ptr mVs;

		// VCO
		const Attribute<Real>::Ptr mVCOOutput;

		// input, state and output vector for logging
		const Attribute<Matrix>::Ptr mVoltagectrlInputs;
		const Attribute<Matrix>::Ptr mVoltagectrlStates;
		const Attribute<Matrix>::Ptr mVoltagectrlOutputs;

		/// Defines name amd logging level
		VSIVoltageControlDQ(String name, Logger::Level logLevel = Logger::Level::off)
			: VSIVoltageControlDQ(name, name, logLevel) {}
		/// Defines UID, name, logging level and connection trafo existence
		VSIVoltageControlDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Setter for general parameters of inverter
		void setParameters(Real sysOmega, Real VdRef, Real VqRef);
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega);
		/// Setter for parameters of transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower,
			Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance, Real omega);
		/// Setter for parameters of filter
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		/// Setter for initial values applied in controllers
		void setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);
		void withControl(Bool controlOn) { mWithControl = controlOn; };

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

		// #### Control section ####
		/// Control pre step operations
		void controlPreStep(Real time, Int timeStepCount);
		/// Perform step of controller
		void controlStep(Real time, Int timeStepCount);
		/// Add control step dependencies
		void addControlPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add control step dependencies
		void addControlStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		class ControlPreStep : public CPS::Task {
		public:
			ControlPreStep(VSIVoltageControlDQ& VSIVoltageControlDQ) :
				Task(**VSIVoltageControlDQ.mName + ".ControlPreStep"), mVSIVoltageControlDQ(VSIVoltageControlDQ) {
					mVSIVoltageControlDQ.addControlPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVSIVoltageControlDQ.controlPreStep(time, timeStepCount); };

		private:
			VSIVoltageControlDQ& mVSIVoltageControlDQ;
		};

		class ControlStep : public CPS::Task {
		public:
			ControlStep(VSIVoltageControlDQ& VSIVoltageControlDQ) :
				Task(**VSIVoltageControlDQ.mName + ".ControlStep"), mVSIVoltageControlDQ(VSIVoltageControlDQ) {
					mVSIVoltageControlDQ.addControlStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVSIVoltageControlDQ.controlStep(time, timeStepCount); };

		private:
			VSIVoltageControlDQ& mVSIVoltageControlDQ;
		};

	};
}
}
}
