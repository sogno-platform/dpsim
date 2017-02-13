#ifndef INDUCTOR_H
#define INDUCTOR_H

#include "BaseComponent.h"

class Inductor : public BaseComponent {
protected:
	double inductance;
	double deltavr;
	double deltavi;
	double currr;
	double curri;
	double cureqr;
	double cureqi;
	double glr, gli;
	double pr, pi;
		
public:
	Inductor() { };
	Inductor(std::string name, int src, int dest, double inductance);

	void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt);
	void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
	void init(int compOffset, double om, double dt);
	void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
	void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);
};
#endif