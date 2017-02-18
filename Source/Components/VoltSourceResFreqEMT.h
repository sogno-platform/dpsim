#ifndef VOLTSOURCERESFREQEMT_H
#define VOLTSOURCERESFREQEMT_H

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceResFreqEMT : public BaseComponent {
	protected:
		Real mVoltageAmp;
		Real mVoltagePhase;
		Real mSwitchTime;
		Real mVoltageDiff;
		Real mResistance;
		Real mConductance;
		Real mCurrent;
		Real mOmegaSource;

	public:
		VoltSourceResFreqEMT() { ; };
		VoltSourceResFreqEMT(std::string name, int src, int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime);

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt);
		void init(int compOffset, Real om, Real dt) { }
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, Real om, Real dt, Real t) { }
	};

}
#endif
