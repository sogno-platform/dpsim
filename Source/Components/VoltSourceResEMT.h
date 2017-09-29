#ifndef VOLTSOURCERESEMT_H
#define VOLTSOURCERESEMT_H

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceResEMT : public BaseComponent {
	protected:
		Complex mVoltage;
		Real mVoltageAmp;
		Real mVoltagePhase;
		Real mVoltageDiff;			
		Real mResistance;
		Real mConductance;
		Real mCurrent;
		
	public:
		VoltSourceResEMT() { ; };
		VoltSourceResEMT(std::string name, int src, int dest, Complex voltage, Real resistance);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system);
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system) { }
	};
}
#endif
#pragma once
