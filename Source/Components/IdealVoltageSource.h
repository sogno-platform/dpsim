#ifndef IDEALVOLTAGESOURCE_H
#define IDEALVOLTAGESOURCE_H

#include "BaseComponent.h"

namespace DPsim {

	class IdealVoltageSource : public BaseComponent {
	protected:
		Real mVoltageDiffr;
		Real mVoltageDiffi;
		Real mVoltageAtSourcer;
		Real mVoltageAtSourcei;
		Real mVoltageAtDestr;
		Real mVoltageAtDesti;
		Real mCurrentr;
		Real mCurrenti;
		int number;		

	public:		
		IdealVoltageSource() { ; };
		IdealVoltageSource(std::string name, int src, int dest, Real voltage, Real phase, int num);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system);
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system) { }
	};
}
#endif
