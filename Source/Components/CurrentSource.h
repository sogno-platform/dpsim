#ifndef CURRENTSOURCE_H
#define CURRENTSOURCE_H

#include "BaseComponent.h"

class CurrentSource : public BaseComponent {
protected:
	double currentr;
	double currenti;

public:
	CurrentSource() {;};
	CurrentSource(std::string name, int src, int dest, double current, double phase);
		
	void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) { }
	void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt);
	void init(int compOffset, double om, double dt) { }
	void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) { }
	void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) { }
};
#endif