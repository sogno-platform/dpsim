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

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt);
		void init(int compOffset, Real om, Real dt) { }
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, Real om, Real dt, Real t) { }
	};
}
#endif
#pragma once
