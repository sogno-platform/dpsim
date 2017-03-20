#ifndef INDUCTOR2_H
#define INDUCTOR2_H

#include "BaseComponent.h"

namespace DPsim {

	class Inductor2 : public BaseComponent {
	protected:
		double inductance;
		double deltavr;
		double deltavi;
		double currr;
		double curri;
		double cureqr;
		double cureqi;
		double glr, gli;
		double pr, pi;


	public:
		Inductor2() { };
		Inductor2(std::string name, int src, int dest, double inductance);

		void init(double om, double dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system);
		void postStep(SystemModel& system);
	};
}
#endif