#ifndef PILINE_H
#define PILINE_H

#include "../Components.h"

namespace DPsim {

	class PiLine : public BaseComponent {
	protected:
		///resistance in ohms
		Real mResistance;

		///Conductance
		Real mConductance;

		///Capacitance
		Real mCapacitance;

		///Inductance
		Real mInductance;

		Real mVoltageAtNode1Re;
		Real mVoltageAtNode1Im;
		Real mVoltageAtNode2Re;
		Real mVoltageAtNode2Im;

		Real mDeltaVre;
		Real mDeltaVim;

		Real mCurrIndRe;
		Real mCurrIndIm;

		Real mCurrCapRe1;
		Real mCurrCapIm1;
		Real mCurrCapRe2;
		Real mCurrCapIm2;


		Real mCurEqIndRe;
		Real mCurEqIndIm;

		Real mCurEqCapRe1;
		Real mCurEqCapIm1;

		Real mCurEqCapRe2;
		Real mCurEqCapIm2;


		Real mGlr;
		Real mGli;
		Real mGcr;
		Real mGci;
		Real mPrevCurFacRe;
		Real mPrevCurFacIm;

	public:
		PiLine() { };
		//PiLine(std::string nam3e, int node1, int node2, Real resistance, Real inductance);
		PiLine(std::string name, int node1, int node2, int node3, Real resistance, Real inductance, Real capacitance);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};
}

#endif