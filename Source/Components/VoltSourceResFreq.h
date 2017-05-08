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
		Real mRampTime;

	public:
		VoltSourceResFreq() { ; };
		VoltSourceResFreq(std::string name, int src, int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime, Real rampTime);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system);
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system) { }
		Complex getCurrent(SystemModel& system);
	};

}
#endif
