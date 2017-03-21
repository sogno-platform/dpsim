#ifndef CAPACITOR_H
#define CAPACITOR_H

#include <iostream>

#include "BaseComponent.h"

namespace DPsim {

	class Capacitor : public BaseComponent {
	protected:
		Real capacitance;
		Real deltavr;
		Real deltavi;
		Real currr;
		Real curri;
		Real cureqr;
		Real cureqi;
		Real mGcr;
		Real mGci;

	public:
		Capacitor() { };
		Capacitor(std::string name, int src, int dest, double capacitance);
		
		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}
#endif