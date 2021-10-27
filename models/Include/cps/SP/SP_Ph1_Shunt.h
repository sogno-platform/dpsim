/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once
#include <cps/SimPowerComp.h>
#include <cps/Solver/PFSolverInterfaceBranch.h>

namespace CPS {

namespace SP {namespace Ph1 {

	class Shunt : public SimPowerComp<Complex>, public SharedFactory<Shunt>, public PFSolverInterfaceBranch {

	private:
		/// Conductance [S]
		Real mConductance;
		/// Susceptance [S]
		Real mSusceptance;
		/// Impedance [Ohm]
		Complex mImpedance;
		/// Addmitance [S]
		Complex mAdmittance;

        /// Base apparent power [VA]
        Real mBaseApparentPower;
        /// Base impedance [Ohm]
        Real mBaseImpedance;
        /// Base admittance [S]
        Real mBaseAdmittance;
        /// Base omega [1/s]
        Real mBaseOmega;
        /// Base voltage [V]
        Real mBaseVoltage;

		/// Conductance [pu]
        Real mConductancePerUnit;
		/// Susceptance [pu]
        Real mSusceptancePerUnit;
		/// Impedance [pu]
        Complex mImpedancePerUnit;
		/// Addmitance [pu]
        Complex mAdmittancePerUnit;

	public:
		/// Defines UID, name, component parameters and logging level
		Shunt(String uid, String name, Logger::Level logLevel = Logger::Level::off);

		/// Defines name and logging level
		Shunt(String name, Logger::Level logLevel = Logger::Level::off)
			: Shunt(name, name, logLevel) { }

		// #### General ####
		/// Set shunt specific parameters
		void setParameters(Real conductance, Real susceptance);

		// #### Powerflow section ####
		/// Set base voltage
		void setBaseVoltage(Real baseVoltage);
		/// Initializes component from power flow data
		void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
		/// Stamps admittance matrix
		void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y);

	};
}
}
}
