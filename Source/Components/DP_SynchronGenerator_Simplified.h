/** Synchron generator
*
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
*
* DPsim
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

#include "Base_SynchronGenerator.h"

namespace DPsim {
		namespace Components {
				namespace DP {

						/// Synchronous generator model
						/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
						/// referred values. The calculation to per unit is performed in the initialization.
						/// The case where parInPerUnit is not set will be implemented later.
						/// parameter names include underscores and typical variables names found in literature instead of
						/// descriptive names in order to shorten formulas and increase the readability

						class SynchronGeneratorSimplified : public SynchronGeneratorBase, public SharedFactory<SynchronGeneratorSimplified> {

						protected:

								/// Determinant of Ld (inductance matrix of d axis)
								Real detLd;
								/// Determinant of Lq (inductance matrix of q axis)
								Real detLq;

								/// Interface voltage phase a _ Real part
								Real mVaRe;
								/// Interface voltage phase a _ Imaginary part
								Real mVaIm;
								/// Interface voltage phase b _ Real part
								Real mVbRe;
								/// Interface voltage phase b _ Imaginary part
								Real mVbIm;
								/// Interface voltage phase c _ Real part
								Real mVcRe;
								/// Interface voltage phase c _ Imaginary part
								Real mVcIm;

								/// Interface current phase a _ Real part
								Real mIaRe;
								/// Interface current phase a _ Imaginary part
								Real mIaIm;
								/// Interface current phase b _ Real part
								Real mIbRe;
								/// Interface current phase b _ Imaginary part
								Real mIbIm;
								/// Interface current phase c _ Real part
								Real mIcRe;
								/// Interface current phase c _ Imaginary part
								Real mIcIm;

								/// reactance matrix
								Matrix mReactanceMat;
								/// omega - flux matrix
								Matrix mOmegaFluxMat;
								/// matrix for reversing stator current directions in calculations with respect to other currents
								Matrix mReverseCurrents;
								/// voltage vector q d 0 kq1 kq2 df kd
								Matrix mVoltages;
								/// flux linkage vector
								Matrix mFluxes;
								/// current vector
								Matrix mCurrents;
								/// Stator Current vector
								Matrix mIabc = Matrix::Zero(6, 1);

								/// Compensation Resistance
								Real mRa;


								Matrix mG_load = Matrix::Zero(6, 6);
								Matrix mR_load = Matrix::Zero(6, 6);
								Matrix mR_eq = Matrix::Zero(2, 2);
								Matrix mE_eq = Matrix::Zero(2, 1);
								Matrix mVdq0 = Matrix::Zero(3, 1);
								

						public:
								~SynchronGeneratorSimplified();

								/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
								/// stator referred parameters depending on the setting of parameter type.
								/// The initialization mode depends on the setting of state type.
								SynchronGeneratorSimplified(String name, Int node1, Int node2, Int node3,
										Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
										Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
										Real Rfd, Real Llfd, Real Rkd, Real Llkd,
										Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
										Real inertia, Real Ra, Logger::Level logLevel = Logger::Level::NONE);

								/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
								/// Function parameters have to be given in Real units.
								void initialize(Real om, Real dt,
										Real initActivePower, Real initReactivePower, Real initTerminalVolt,
										Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

								/// Performs an Euler forward step with the state space model of a synchronous generator
								/// to calculate the flux and current from the voltage vector.
								void step(SystemModel& system, Real time);

								/// Performs an Euler forward step with the state space model of a synchronous generator
								/// to calculate the flux and current from the voltage vector in per unit.
								void stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod);

								/// Retrieves calculated voltage from simulation for next step
								void postStep(SystemModel& system);

								/// Park transform as described in Krause
								//Matrix abcToDq0Transform(Real theta, Matrix& in);
								Matrix abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm);

								/// Inverse Park transform as described in Krause
								//Matrix dq0ToAbcTransform(Real theta, Matrix& in);
								Matrix dq0ToAbcTransform(Real theta, Real d, Real q, Real zero);

								Matrix& getVoltages() { return mVoltages; }
								Matrix& getCurrents() { return mCurrents; }
								Matrix& getFluxes() { return mFluxes; }
								Matrix& getStatorCurrents() { return mIabc; }
								Real getElectricalTorque() { return mElecTorque*mBase_T; }
								Real getRotationalSpeed() { return mOmMech*mBase_OmMech; }
								Real getRotorPosition() { return mThetaMech; }


								void initialize(SystemModel& system) { }
								void applySystemMatrixStamp(SystemModel& system);
								void applyRightSideVectorStamp(SystemModel& system) { }
						};
				}
		}
}
