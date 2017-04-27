#pragma once

namespace DPsim {
	/** Abstract base class for interfacing the simulator with other data sources or sinks.
	 * After an ExternalInterface is created, components that should use values
	 * from this interface can be registered with it using the appropiate
	 * methods implemented by the subclass. Subclasses must also implement the
	 * readValues method, which should update the values of the registered
	 * components.
	 */
	class ExternalInterface {
	public:
		virtual void readValues() = 0;
	};
}
