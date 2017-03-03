#ifndef IDEALVOLTAGESOURCE_H
#define IDEALVOLTAGESOURCE_H

#include "BaseComponent.h"

namespace DPsim {

	class IdealVoltageSource : public BaseComponent {
	protected:
		Real mVoltageDiffr;
		Real mVoltageDiffi;
		Real mVoltageAtSourcer;
		Real mVoltageAtSourcei;
		Real mVoltageAtDestr;
		Real mVoltageAtDesti;
		Real mCurrentr;
		Real mCurrenti;

	public:
		IdealVoltageSource() { ; };
		IdealVoltageSource(std::string name, int src, int dest, Real voltage, Real phase);

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt);
		void init(int compOffset, Real om, Real dt) { }
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, Real om, Real dt, Real t) { }
	};
}
#endif
