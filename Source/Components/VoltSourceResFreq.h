#ifndef VOLTSOURCERESFREQ_H
#define VOLTSOURCERESFREQ_H

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceResFreq : public BaseComponent {
	protected:
		Real mVoltageAmp;
		Real mVoltagePhase;
		Real mSwitchTime;
		Real mVoltageDiffr;
		Real mVoltageDiffi;
		Real mVoltageAtSourcer;
		Real mVoltageAtSourcei;
		Real mVoltageAtDestr;
		Real mVoltageAtDesti;
		Real mResistance;
		Real mConductance;
		Real mCurrentr;
		Real mCurrenti;
		Real mOmegaSource;

	public:
		VoltSourceResFreq() { ; };
		VoltSourceResFreq(std::string name, int src, int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime);

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt);
		void init(int compOffset, Real om, Real dt) { }
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, Real om, Real dt, Real t) { }
	};

}
#endif
