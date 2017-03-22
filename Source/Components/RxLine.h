#ifndef RXLINE_H
#define RXLINE_H

#include "BaseComponent.h"

<<<<<<< HEAD
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
		
		double correctr, correcti;
		double cureqr_ind, cureqi_ind;
		double deltavr_ind;
		double deltavi_ind;
		double glr_ind, gli_ind;
		double currr_ind;
		double curri_ind;

	public:
		RxLine() { };
		RxLine(std::string name, int src, int dest, int node3, Real resistance, Real inductance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}

#endif