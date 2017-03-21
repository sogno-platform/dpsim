#ifndef LINEARRESISTOREMT_H
#define LINEARRESISTOREMT_H

#include <iostream>
#include "BaseComponent.h"

namespace DPsim {

	class LinearResistorEMT : public BaseComponent {
	protected:
		double mResistance;
		double mConductance;
		double mVoltageAtNode1;
		double mVoltageAtNode2;

	public:
		LinearResistorEMT() { ; };
		LinearResistorEMT(std::string name, int src, int dest, Real resistance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}
#endif
