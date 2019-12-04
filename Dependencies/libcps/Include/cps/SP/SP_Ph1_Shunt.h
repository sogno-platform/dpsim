/**
 * @file
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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
#include <cps/Solver/PFSolverInterfaceBranch.h>

namespace CPS {

namespace SP {namespace Ph1 {

	class Shunt : public PowerComponent<Complex>, public SharedFactory<Shunt>, public PFSolverInterfaceBranch {

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
