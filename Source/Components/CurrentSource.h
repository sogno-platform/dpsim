#ifndef CURRENTSOURCE_H
#define CURRENTSOURCE_H

#include "BaseComponent.h"

class CurrentSource : public BaseComponent
{
	public:
		CurrentSource() {;};
		CurrentSource(std::string name, int src, int dest, double current, double phase);
		
		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
		
	protected:
		double currentr;
		double currenti;
};
#endif