#ifndef INTERFACEDINDUCTOR_H
#define INTERFACEDINDUCTOR_H

#include "BaseComponent.h"

namespace DPsim {

	class InterfacedInductor : public BaseComponent {
	protected:
		double mInductance;
		double mVoltageRe;
		double mVoltageIm;
		double mCurrentRe;
		double mCurrentIm;
		double mCurrentStepRe;
		double mCurrentStepIm;

	public:
		InterfacedInductor() { };
		InterfacedInductor(std::string name, int src, int dest, Real inductance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}
#endif
