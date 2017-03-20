#ifndef LINEARRESISTOREMT_H
#define LINEARRESISTOREMT_H

#include <iostream>
#include "BaseComponent.h"

namespace DPsim {

	class LinearResistorEMT : public BaseComponent {
	protected:
		double resistance;
		double conductance;
		double voltageAtSourcer;
		double voltageAtSourcei;

		double voltageAtDestr;
		double voltageAtDesti;

	public:
		LinearResistorEMT() { ; };
		LinearResistorEMT(std::string name, int src, int dest, double resistance);

		void init(double om, double dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system);
		void postStep(SystemModel& system);
	};
}
#endif
