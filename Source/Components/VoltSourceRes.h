#ifndef VOLTSOURCERES_H
#define VOLTSOURCERES_H

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceRes : public BaseComponent {
	protected:
		Real mVoltageDiffr;
		Real mVoltageDiffi;
		Real mVoltageAtSourcer;
		Real mVoltageAtSourcei;
		Real mVoltageAtDestr;
		Real mVoltageAtDesti;
		Real mResistance;
		Real mConductance;
		Real mCurrentr;
		Real mCurrenti;

	public:
		VoltSourceRes() { ; };
		VoltSourceRes(std::string name, int src, int dest, Real voltage, Real phase, Real resistance);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system) { }
	};
}
#endif
