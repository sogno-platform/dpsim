/** External interface
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

#include <vector>

#include "Components/Base_ExportableCurrent.h"
#include "Components/Base_ControllableSource.h"

namespace DPsim {

	/** \brief Abstract base class for interfacing the simulator with other data sources or sinks.
	 *
	 * After an ExternalInterface is created, components that should use values
	 * from this interface can be registered with it using the appropiate
	 * methods implemented by the subclass. Subclasses must also implement the
	 * readValues and writeValues methods, which should update the values of
	 * the registered components or send voltages or currents to the external
	 * sink.
	 */
	class ExternalInterface {

	protected:
		struct ControllableSourceMapping {
			Components::ControllableSourceBase::Ptr comp;
			Int realIdx;
			Int imagIdx;
		};

		struct ExportableCurrentMapping {
			Components::ExportableCurrentBase::Ptr comp;
			Int realIdx;
			Int imagIdx;
		};

		struct ExportableVoltageMapping {
			Int from;
			Int to;
			Int realIdx;
			Int imagIdx;
		};

		std::vector<ControllableSourceMapping> mControllableSources;

		std::vector<ExportableCurrentMapping> mExportedCurrents;
		std::vector<ExportableVoltageMapping> mExportedVoltages;
		bool mInit = 0;

	public:
		/** Register an external voltage source to use values from this interface.
		 * @param vs The external voltage source to register.
		 * @param realIdx Interface-specific index identifying the real part.
		 * @param imagIdx Interface-specific index identifying the imaginary part.
		 */
		void registerControllableSource(Components::ControllableSourceBase::Ptr src, Int realIdx, Int imagIdx);

		/** Register a voltage between two nodes to be sent through this
		 * interface after every step.
		 * @param from Number of the node used as the positive potential.
		 * Node numbers are like passed to the component constructors (starting
		 * from 1 with 0 meaning ground).
		 * @param to Number of the nodes used as the negative potential.
		 * @param realIdx Interface-specific index identifying the real part.
		 * @param imagIdx Interface-specific index identifying the imaginary part.
		 */
		void registerExportedVoltage(Int from, Int to, Int realIdx, Int imagIdx);

		/** Register a current through a component to be sent through this
		 * interface after every step.
		 * @param comp Component whose current is sent. Note that this component
		 * must properly implement the getCurrent method.
		 * @param realIdx Interface-specific index identifying the real part.
		 * @param imagIdx Interface-specific index identifying the imaginary part.
		 */
		void registerExportedCurrent(Components::ExportableCurrentBase::Ptr comp, Int realIdx, Int imagIdx);

		/** Read data for a timestep from the interface and passes the values
		 * to all registered current / voltage sources.
		 */
		virtual void readValues(bool blocking = true) = 0;

		/** Write all exported values to the interface. Called after every timestep.
		 * @param model Reference to the system model which should be used to
		 * calculate needed voltages.
		 */

		virtual void writeValues(SystemModel &model) = 0;
		virtual ~ExternalInterface() { };
	};
}
