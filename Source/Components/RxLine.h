#ifndef RXLINE_H
#define RXLINE_H

#include "BaseComponent.h"

class RxLine : public BaseComponent {
protected:

	double resistance;
	double conductance;
	double voltageAtSourcer;
	double voltageAtSourcei;
	double voltageAtDestr;
	double voltageAtDesti;

	double inductance;
	double deltavr;
	double deltavi;
	double currr;
	double curri;
	double cureqr;
	double cureqi;
	double glr, gli;
	double pr, pi;

	double correctr, correcti;
	double cureqr_ind, cureqi_ind;
	double deltavr_ind;
	double deltavi_ind;
	double glr_ind, gli_ind;
	double currr_ind;
	double curri_ind;

	//int newnode;


public:
	RxLine() { };
	RxLine(std::string name, int src, int dest, int node3, double resistance, double inductance);
	//RxLine(std::string name, int src, int dest, double resistance, double inductance);

	void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt);
	void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
	void init(double om, double dt);
	void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
	void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);

};
#endif