#include "Inductor.h"

using namespace DPsim;

Inductor::Inductor(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->inductance = inductance;
}	
		
void Inductor::applySystemMatrixStamp(SystemModel& system) {
	double a = system.getTimeStep() / (2. * inductance);
	double b = system.getTimeStep() * system.getOmega() / 2.;
	mGlr = a / (1 + b*b);
	mGli = -a*b / (1 + b*b);
	mPrevCurFacRe = (1 - b*b) / (1 + b*b);
	mPrevCurFacIm = - 2 * b / (1 + b*b);
			 
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mGlr, mGli);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGlr, mGli);
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, -mGlr, -mGli);
		system.addCompToSystemMatrix(mNode2, mNode1, -mGlr, -mGli);
	}
}


void Inductor::init(Real om, Real dt) {
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}


void Inductor::step(SystemModel& system) {
	// Initialize internal state
	cureqr = mGlr * deltavr - mGli * deltavi + pr * currr - pi * curri;
	cureqi = mGli * deltavr + mGlr * deltavi + pi * currr + pr * curri;
		
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) - cureqr;
		j(compOffset+mNode1, 0) = j(compOffset+mNode1, 0) - cureqi;
	}

	if (mNode2 >= 0)	{
		j(mNode2, 0) = j(mNode2, 0) + cureqr;
		j(compOffset+mNode2, 0) = j(compOffset+mNode2, 0) + cureqi;
	}
}


void Inductor::postStep(SystemModel& system) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (mNode1 >= 0)	{
		vposr = vt(mNode1, 0);
		vposi = vt(compOffset+mNode1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}
	
	if (mNode2 >= 0) {
		vnegr = vt(mNode2, 0);
		vnegi = vt(compOffset+mNode2, 0);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	deltavr = vposr-vnegr;
	deltavi = vposi-vnegi;
	currr =  glr*deltavr-gli*deltavi+cureqr;
	curri =  gli*deltavr+glr*deltavi+cureqi;
}
