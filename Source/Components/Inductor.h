#ifndef INDUCTOR_H
#define INDUCTOR_H

#include "BaseComponent.h"

class Inductor : public BaseComponent {

	/// Inductor model:
	/// The inductor is represented by a DC equivalent circuit which corresponds to one iteration of the trapezoidal integration method.
	/// The equivalent DC circuit is a resistance in paralel with a current source. The resistance is constant for a defined time step and system
	///frequency and the current source changes for each iteration.

protected:

	//  ### Indcutor parameters ###
	/// Inductance [H]
	double inductance;

	/// Real and imaginary part of the voltage across the inductor [V]
	double deltavr;
	double deltavi;

	/// Real and imaginary part of the current trough the inductor [A]
	double currr;
	double curri;

	/// Real and imaginary part of the DC equivalent current source [A]
	double cureqr;
	double cureqi;

	/// Real and imaginary part of the DC equivalent conductance [S]
	double glr, gli;

	/// Auxiliar variables
	double pr, pi;
		
public:
	Inductor() { };

	/// Define inductor name, conected nodes and inductance 
	Inductor(std::string name, int src, int dest, double inductance);

	/// Stamps DC equivalent resistance to the conductance matrix
	void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt);

	/// Stamps DC equivalent current source to the current vector
	void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }

	/// Initializes variables detalvr, deltavi, currr, curri, cureqr and curreqi
	void init(double om, double dt);

	/// calculates the value of the current source for one time step and apply it to the current vector
	void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);

	/// Recalculates variables detalvr, deltavi, currr and curri based on the simulation results of one time step
	void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);
};
#endif