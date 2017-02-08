#ifndef INDUCTOR2_H
#define INDUCTOR2_H

#include "BaseComponent.h"

class Inductor2 : public BaseComponent {
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
	Inductor2() { };
	Inductor2(std::string name, int src, int dest, double inductance);

	void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
	void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
	void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
	void PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);
};
#endif