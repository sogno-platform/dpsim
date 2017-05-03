#pragma once

#include <vector>

#include "Components/ExternalCurrentSource.h"
#include "Components/ExternalVoltageSource.h"

namespace DPsim {
	struct VoltDiff {
		int from;
		int to;
	};
	/** Abstract base class for interfacing the simulator with other data sources or sinks.
	 * After an ExternalInterface is created, components that should use values
	 * from this interface can be registered with it using the appropiate
	 * methods implemented by the subclass. Subclasses must also implement the
	 * readValues method, which should update the values of the registered
	 * components.
	 */
	class ExternalInterface {
	protected:
		std::vector<BaseComponent*> mExtComponents;
		std::vector<VoltDiff> mExportedVoltages;
	public:
		void registerVoltageSource(ExternalVoltageSource* evs, int num);
		void registerCurrentSource(ExternalCurrentSource* ecs, int num);
		void registerExportedVoltage(VoltDiff vd, int num);
		virtual void readValues() = 0;
		virtual void writeValues(SystemModel &model) = 0;
		virtual ~ExternalInterface() {};
	};
}
