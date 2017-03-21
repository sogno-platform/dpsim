#ifndef LINEARRESISTOR_H
#define LINEARRESISTOR_H

#include <iostream>
#include "BaseComponent.h"

namespace DPsim {

	class LinearResistor : public BaseComponent {
	protected:
		Real mResistance;
		Real mConductance;
		Real mVoltageAtNode1Re;
		Real mVoltageAtNode1Im;

		Real mVoltageAtNode2Re;
		Real mVoltageAtNode2Im;

	public:
		LinearResistor() { ; };
		LinearResistor(std::string name, int src, int dest, Real resistance);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time) { }
		void postStep(SystemModel& system) { }
	};
}
#endif
