#ifndef VOLTSOURCERESEMT_H
#define VOLTSOURCERESEMT_H

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceResEMT : public BaseComponent {
	protected:
		Real mVoltageAmp;
		Real mVoltagePhase;
		Real mVoltageDiff;			
		Real mResistance;
		Real mConductance;
		Real mCurrent;
		

	public:
		VoltSourceResEMT() { ; };
		VoltSourceResEMT(std::string name, int src, int dest, Real voltage, Real phase, Real resistance);

		void init(double om, double dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system);
		void postStep(SystemModel& system);
	};
}
#endif
#pragma once
