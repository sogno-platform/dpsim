#ifndef RXLINE_H
#define RXLINE_H

#include "BaseComponent.h"

namespace DPsim {

	class RxLine : public BaseComponent {
	protected:
		Real mResistance;
		Real mConductance;
		Real mVoltageAtNode1Re;
		Real mVoltageAtNode1Im;
		Real mVoltageAtNode2Re;
		Real mVoltageAtNode2Im;

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
		
		Int newnode;

	public:
		RxLine() { };
		RxLine(std::string name, int src, int dest, int newNode, Real resistance, Real inductance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}
#endif