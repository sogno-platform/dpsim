/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Signal/SignalGenerator.h>
#include <dpsim-models/Signal/SineWaveGenerator.h>
#include <dpsim-models/Signal/FrequencyRampGenerator.h>
#include <dpsim-models/Signal/CosineFMGenerator.h>

namespace CPS {
	namespace EMT {
		namespace Ph3 {
			/// \brief Ideal Voltage source model
			///
			/// This model uses modified nodal analysis to represent an ideal voltage source.
			/// For a voltage source between nodes j and k, a new variable (current across the voltage source)
			/// is added to the left side vector
			/// as unkown and it is taken into account for the equation of node j as positve and for the equation
			/// of node k as negative. Moreover
			/// a new equation ej - ek = V is added to the problem.
			class VoltageSource :
				public MNASimPowerComp<Real>,
				public DAEInterface,
				public SharedFactory<VoltageSource> {
			private:
				///
				CPS::Signal::SignalGenerator::Ptr mSrcSig;
				
			public:
				const CPS::Attribute<MatrixComp>::Ptr mVoltageRef;
				const CPS::Attribute<Real>::Ptr mSrcFreq;

				/// Defines UID, name and logging level
				VoltageSource(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				///
				VoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
					: VoltageSource(name, name, logLevel) { }

				SimPowerComp<Real>::Ptr clone(String name);
				// #### General ####
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency);
				/// Setter for reference voltage and frequency with a sine wave generator
				void setParameters(MatrixComp voltageRef, Real srcFreq = 50.0);
				/// Setter for reference signal of type frequency ramp
				void setParameters(MatrixComp voltageRef, Real freqStart, Real rocof, Real timeStart, Real duration, bool smoothRamp = true);
				/// Setter for reference signal of type cosine frequency modulation
				void setParameters(MatrixComp voltageRef, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency = 50.0, bool zigzag = false);

				// #### MNA section ####
				/// Initializes internal variables of the component
				void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
				/// Stamps system matrix
				void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
				/// Stamps right side (source) vector
				void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
				/// Returns current through the component
				void mnaCompUpdateCurrent(const Matrix& leftVector) override;
				/// MNA pre step operations
				void mnaCompPreStep(Real time, Int timeStepCount) override;
				/// MNA post step operations
				void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
				/// Add MNA pre step dependencies
				void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
				/// Add MNA post step dependencies
				void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

				// #### DAE Section #### (Not yet implemented!!!)
				
				/// Derivative of the current
				MatrixVar<Real> mIntfDerCurrent;
				MatrixVar<Real> intfDerCurrent() {return mIntfDerCurrent;}
				/// set init value of the current, calculate and set the 
				/// initial value of the derivative of the current
				void setInitialComplexIntfCurrent(Complex initCurrent);
				///
				void daeInitialize(double time, double state[], double dstate_dt[],
					double absoluteTolerances[], double stateVarTypes[], int& offset) override;
				/// Residual function for DAE Solver
				void daeResidual(double time, const double state[], const double dstate_dt[], 
					double resid[], std::vector<int>& off) override;
				/// Calculation of jacobian
				void daeJacobian(double current_time, const double state[], const double dstate_dt[], 
					SUNMatrix jacobian, double cj, std::vector<int>& off) override;
				///
				void daePostStep(double Nexttime, const double state[], 
					const double dstate_dt[], int& offset) override;
				///
				int getNumberOfStateVariables() override {return 3;}

			};
		}
	}
}
