#ifndef INTERFACEDINDUCTOR_H
#define INTERFACEDINDUCTOR_H

#include "BaseComponent.h"

namespace DPsim {

	class InterfacedInductor : public BaseComponent {
	protected:
		double inductance;
		double voltageRe;
		double voltageIm;
		double currentRe;
		double currentIm;
		double currentStepRe;
		double currentStepIm;

	public:
		InterfacedInductor() { };
		InterfacedInductor(std::string name, int src, int dest, double inductance);

		void init(double om, double dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system);
		void postStep(SystemModel& system);
	};
}
#endif
