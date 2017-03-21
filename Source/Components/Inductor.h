#ifndef INDUCTOR_H
#define INDUCTOR_H

#include "BaseComponent.h"

namespace DPsim {

	class Inductor : public BaseComponent {
	protected:
		Real mInductance;
		Real mDeltaVre;
		Real mDeltaVim;
		Real mCurrRe;
		Real mCurrIm;
		Real mCurEqRe;
		Real mCurEqIm;
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