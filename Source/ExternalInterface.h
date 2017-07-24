#pragma once

#include <vector>

#include "Components/ExternalCurrentSource.h"
#include "Components/ExternalVoltageSource.h"

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
		/// Internal struct for storing an exported voltage.
		struct VoltDiff {
			Int from;
			Int to;
			Int realIdx;
			Int imagIdx;
		};
		/// Internal struct for storing references to external components.
		struct ExtComponent {
			BaseComponent *comp;
			Int realIdx;
			Int imagIdx;
		};
		std::vector<ExtComponent> mExtComponents;
		std::vector<VoltDiff> mExportedVoltages;
		std::vector<ExtComponent> mExportedCurrents;
		bool mInit = 0;
	public:
		/** Register an external voltage source to use values from this interface.
		 * @param evs The external voltage source to register.
		 * @param realIdx Interface-specific index identifying the real part.
		 * @param imagIdx Interface-specific index identifying the imaginary part.
		 */
		void registerVoltageSource(ExternalVoltageSource* evs, Int realIdx, Int imagIdx);
		/** Register an external current source to use values from this interface.
		 * @param evs The external current source to register.
		 * @param realIdx Interface-specific index identifying the real part.
		 * @param imagIdx Interface-specific index identifying the imaginary part.
		 */
		void registerCurrentSource(ExternalCurrentSource* ecs, Int realIdx, Int imagIdx);
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
		void registerExportedCurrent(BaseComponent *comp, Int realIdx, Int imagIdx);
		/** Read data for a timestep from the interface and passes the values
		 * to all registered current / voltage sources.
		 */
		virtual void readValues(bool blocking = true) = 0;
		/** Write all exported values to the interface. Called after every timestep.
		 * @param model Reference to the system model which should be used to
		 * calculate needed voltages.
		 */
		virtual void writeValues(SystemModel &model) = 0;
		virtual ~ExternalInterface() {};
	};
}
