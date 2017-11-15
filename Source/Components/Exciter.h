/** Exciter
*
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
* @license GNU General Public License (version 3)
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

#include "BaseComponent.h"

namespace DPsim {

	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability

	class Exciter {
	protected:

		Real mTa;
		Real mKa;
		Real mKe;
		Real mTe;
		Real mKf;
		Real mTf;
		Real mTr;

		Real mLad;
		Real mRfd;

		// Reference voltage
		Real mVref = 0;
		// Output of voltage transducer
		Real mVm = 0;
		// Input of voltage transducer
		Real mVh = 0;
		// Output of stablizing feedback
		Real mVis = 0;
		// Output of se function
		Real mVse = 0;
		// Regulator output
		Real mVr = 0;
		// Exciter output
		Real mVf = 0;
		Real mVfd;

		bool mLogActive;
		Logger* mLog;


	public:
		Exciter() { };
		~Exciter();

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		Exciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd, Real Vf_init);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type.
		/// Function parameters have to be given in real units.
		//void init();
		//void init(Real mVd, Real mVq, Real Vfd, Real Vref, Real mLad, Real mRfd);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector.
		void step(Real mVd, Real mVq, Real Vref, Real dt, Real time);

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector in per unit.
		//void stepInPerUnit(Real mVd, Real mVq, Real Vref, Real dt);

		/// Retrieves calculated voltage from simulation for next step
		//void postStep(SystemModel& system);

		//void init(Real om, Real dt) { }
		//void applySystemMatrixStamp(SystemModel& system) { }
		//void applyRightSideVectorStamp(SystemModel& system) { }

		Real UpdateFieldVoltage() { return mVfd; }
	
	};
}
