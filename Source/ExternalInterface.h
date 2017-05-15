#pragma once

#include <vector>

#include "Components/ExternalCurrentSource.h"
#include "Components/ExternalVoltageSource.h"

namespace DPsim {
	struct VoltDiff {
		Int from;
		Int to;
		Int realIdx;
		Int imagIdx;
	};
	struct ExtComponent {
		BaseComponent *comp;
		Int realIdx;
		Int imagIdx;
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
		std::vector<ExtComponent> mExtComponents;
		std::vector<VoltDiff> mExportedVoltages;
		std::vector<ExtComponent> mExportedCurrents;
		bool mInit = 0;
	public:
		void registerVoltageSource(ExternalVoltageSource* evs, Int realIdx, Int imagIdx);
		void registerCurrentSource(ExternalCurrentSource* ecs, Int realIdx, Int imagIdx);
		void registerExportedVoltage(Int from, Int to, Int realIdx, Int imagIdx);
		void registerExportedCurrent(BaseComponent *comp, Int realIdx, Int imagIdx);
		virtual void readValues() = 0;
		virtual void writeValues(SystemModel &model) = 0;
		virtual ~ExternalInterface() {};
	};
}
