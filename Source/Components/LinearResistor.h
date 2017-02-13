#ifndef LINEARRESISTOR_H
#define LINEARRESISTOR_H

#include <iostream>

#include "BaseComponent.h"

class LinearResistor : public BaseComponent {
protected:
	double resistance;
	double conductance;
	double voltageAtSourcer;
	double voltageAtSourcei;

	double voltageAtDestr;
	double voltageAtDesti;

public:
	LinearResistor() {;};
	LinearResistor(std::string name, int src, int dest, double resistance);

	void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt);
	void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
	void init(int compOffset, double om, double dt) { }
	void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) { }
	void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) { }
};
#endif
