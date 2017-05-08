#ifndef RXLINE_H
#define RXLINE_H

#include "../Components.h"

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

		LineTypes type;
		
		double correctr, correcti;
		double cureqr_ind, cureqi_ind;
		double deltavr_ind;
		double deltavi_ind;
		double glr_ind, gli_ind;
		double currr_ind;
		double curri_ind;

	public:
		RxLine() { };
		RxLine(std::string name, int node1, int node2, Real resistance, Real inductance);
		RxLine(std::string name, int node1, int node2, int node3, Real resistance, Real inductance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
		Complex getCurrent(SystemModel& system);
	};
}

#endif
