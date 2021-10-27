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
#include <cps/Definitions.h>
#include <cps/SP/SP_Ph1_Resistor.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Capacitor.h>
#include <cps/SP/SP_Ph1_VoltageSource.h>
#include <cps/SP/SP_Ph1_Transformer.h>
#include <cps/Base/Base_AvVoltageSourceInverterDQ.h>
#include <cps/Signal/PLL.h>
#include <cps/Signal/PowerControllerVSI.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	class AvVoltageSourceInverterDQ :
		public Base::AvVoltageSourceInverterDQ,
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<AvVoltageSourceInverterDQ> {
	protected:

		// ### General Parameters ###
		/// Nominal system angle
		Real mThetaN = 0;
		/// Nominal frequency
		Real mOmegaN;
		/// Nominal voltage
		Real mVnom;
		/// Simulation step
		Real mTimeStep;
		/// Active power reference
		Real mPref;
		/// Reactive power reference
		Real mQref;

		// ### Control Subcomponents ###
		/// PLL
		std::shared_ptr<Signal::PLL> mPLL;
		/// Power Controller
		std::shared_ptr<Signal::PowerControllerVSI> mPowerControllerVSI;

		// ### Electrical Subcomponents ###
		/// Controlled voltage source
		std::shared_ptr<SP::Ph1::VoltageSource> mSubCtrledVoltageSource;
		/// Resistor Rf as part of LCL filter
		std::shared_ptr<SP::Ph1::Resistor> mSubResistorF;
		/// Capacitor Cf as part of LCL filter
		std::shared_ptr<SP::Ph1::Capacitor> mSubCapacitorF;
		/// Inductor Lf as part of LCL filter
		std::shared_ptr<SP::Ph1::Inductor> mSubInductorF;
		/// Resistor Rc as part of LCL filter
		std::shared_ptr<SP::Ph1::Resistor> mSubResistorC;
		/// Optional connection transformer
		std::shared_ptr<SP::Ph1::Transformer> mConnectionTransformer;

		// ### Inverter Interfacing Variables ###
		// Control inputs
		/// Measured voltage d-axis in local reference frame
		Real mVcd = 0;
		/// Measured voltage q-axis in local reference frame
		Real mVcq = 0;
		/// Measured current d-axis in local reference frame
		Real mIrcd = 0;
		/// Measured current q-axis in local reference frame
		Real mIrcq = 0;
		// Control outputs
		/// Voltage as control output after transformation interface
		MatrixComp mVsref = MatrixComp::Zero(1,1);

		/// Flag for connection transformer usage
		Bool mWithConnectionTransformer=false;
		/// Flag for controller usage
		Bool mWithControl=true;

		// #### solver ####
		///
		std::vector<const Matrix*> mRightVectorStamps;

	public:
		/// Defines name amd logging level
		AvVoltageSourceInverterDQ(String name, Logger::Level logLevel = Logger::Level::off)
			: AvVoltageSourceInverterDQ(name, name, logLevel) {}
		/// Defines UID, name, logging level and connection trafo existence
		AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel = Logger::Level::off, Bool withTrafo = false);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Setter for general parameters of inverter
		void setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref);
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff);
		/// Setter for parameters of transformer
		void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,	Real ratioPhase, Real resistance, Real inductance);
		/// Setter for parameters of filter
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
		/// Setter for initial values applied in controllers
		void setInitialStateValues(Real pInit, Real qInit,
			Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);
		void withControl(Bool controlOn) { mWithControl = controlOn; };

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Updates current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates voltage across component
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

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
			ControlPreStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".ControlPreStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
					mAvVoltageSourceInverterDQ.addControlPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mAvVoltageSourceInverterDQ.controlPreStep(time, timeStepCount); };

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class ControlStep : public CPS::Task {
		public:
			ControlStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".ControlStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
					mAvVoltageSourceInverterDQ.addControlStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mAvVoltageSourceInverterDQ.controlStep(time, timeStepCount); };

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPreStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
					mAvVoltageSourceInverterDQ.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mAvVoltageSourceInverterDQ.mnaPreStep(time, timeStepCount); };

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(AvVoltageSourceInverterDQ& AvVoltageSourceInverterDQ, Attribute<Matrix>::Ptr leftVector) :
				Task(AvVoltageSourceInverterDQ.mName + ".MnaPostStep"), mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ), mLeftVector(leftVector) {
				mAvVoltageSourceInverterDQ.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mAvVoltageSourceInverterDQ.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			AvVoltageSourceInverterDQ& mAvVoltageSourceInverterDQ;
			Attribute<Matrix>::Ptr mLeftVector;
		};

	};
}
}
}
