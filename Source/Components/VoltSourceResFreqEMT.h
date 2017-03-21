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
		Real mRampTime;

	public:
		VoltSourceResFreqEMT() { ; };
		VoltSourceResFreqEMT(std::string name, int src, int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime, Real rampTime);

		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system);
	};

}
#endif
