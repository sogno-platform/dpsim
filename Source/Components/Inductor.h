#ifndef INDUCTOR_H
#define INDUCTOR_H

#include "BaseComponent.h"

namespace DPsim {

	class Inductor : public BaseComponent {
	protected:
		Real inductance;
		Real deltavr;
		Real deltavi;
		Real currr;
		Real curri;
		Real cureqr;
		Real cureqi;
		Real mGlr;
		Real mGli;
		Real mPrevCurFacRe;
		Real mPrevCurFacIm;

	public:
		Inductor() { };
		Inductor(std::string name, int src, int dest, double inductance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system);
		void postStep(SystemModel& system);
	};
}
#endif