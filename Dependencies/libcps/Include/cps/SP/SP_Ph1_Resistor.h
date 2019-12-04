/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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
#include <cps/Solver/PFSolverInterfaceBranch.h>
#include <cps/Definitions.h>
#include <cps/Logger.h>
#include <cps/Base/Base_Ph1_Resistor.h>

namespace CPS {
	namespace SP {
		namespace Ph1 {
			///
			class Resistor :
				public Base::Ph1::Resistor,
				public MNATearInterface,
				public PowerComponent<Complex>,
				public SharedFactory<Resistor>,
	 			public PFSolverInterfaceBranch {
			
			private:
				/// base apparent power[VA]
				Real mBaseApparentPower;
				/// base impedance [ohm]
				Real mBaseImpedance;
				/// base admittance [S]
				Real mBaseAdmittance;
				/// base voltage [V]
				Real mBaseVoltage;
				/// base current [A]
				Real mBaseCurrent;

				/// resistance [pu]
				Real mResistancePerUnit;
				/// conductance [pu]
				Real mConductancePerUnit;


			public:
				/// Defines UID, name and logging level
				Resistor(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				/// Defines name and logging level
				Resistor(String name, Logger::Level logLevel = Logger::Level::off)
					: Resistor(name, name, logLevel) { }

				PowerComponent<Complex>::Ptr clone(String name);

				// #### General ####
				/// Initializes component from power flow data
				void initializeFromPowerflow(Real frequency);

				 // #### Powerflow section ####
				 /// Set base voltage
				void setBaseVoltage(Real baseVoltage);
				/// Initializes component from power flow data
				void calculatePerUnitParameters(Real baseApparentPower);
				/// Stamps admittance matrix
				void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y);

				// #### MNA section ####
				///
				void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				/// Stamps system matrix
				void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				///
				void mnaUpdateVoltage(const Matrix& leftVector);
				///
				void mnaUpdateCurrent(const Matrix& leftVector);

				class MnaPostStep : public Task {
				public:
					MnaPostStep(Resistor& resistor, Attribute<Matrix>::Ptr leftSideVector) :
						Task(resistor.mName + ".MnaPostStep"), mResistor(resistor), mLeftVector(leftSideVector)
					{
						mAttributeDependencies.push_back(mLeftVector);
						mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
						mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
					}

					void execute(Real time, Int timeStepCount);

				private:
					Resistor& mResistor;
					Attribute<Matrix>::Ptr mLeftVector;
				};
				// #### MNA Tear Section ####
				void mnaTearApplyMatrixStamp(Matrix& tearMatrix);

			};
		}
	}
}
