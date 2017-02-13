#ifndef INTERFACEDINDUCTOR_H
#define INTERFACEDINDUCTOR_H

#include "BaseComponent.h"

class InterfacedInductor : public BaseComponent {
protected:
	double inductance;
	double voltageRe;
	double voltageIm;
	double currentRe;
	double currentIm;
	double currentStepRe;
	double currentStepIm;

public:
	InterfacedInductor() { };
	InterfacedInductor(std::string name, int src, int dest, double inductance);
		
	void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) { }
	void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
	void init(int compOffset, double om, double dt);
	void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
	void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);
};
#endif
