#ifndef VOLTSOURCERES_H
#define VOLTSOURCERES_H

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceRes : public BaseComponent {
	protected:
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

	public:
		VoltSourceRes() { ; };
		VoltSourceRes(std::string name, int src, int dest, Real voltage, Real phase, Real resistance);

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt);
		void init(int compOffset, Real om, Real dt) { }
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, Real om, Real dt, Real t) { }
	};
}
#endif
