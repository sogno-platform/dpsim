#ifndef CAPACITOR_H
#define CAPACITOR_H

#include <iostream>

#include "BaseComponent.h"


namespace DPsim {
	
	/// Capacitor model:
	/// The capacitor is represented by a DC equivalent circuit which corresponds to one iteration of the trapezoidal integration method.
	/// The equivalent DC circuit is a resistance in paralel with a current source. The resistance is constant for a defined time step and system
	///frequency and the current source changes for each iteration.
	class Capacitor : public BaseComponent {
	protected:
		/// Capacitance [F]
		Real capacitance;
		
		/// Real part of the voltage across the capacitor [V]
		Real deltavr;

		/// Imaginary part of the voltage across the capacitor [V]
		Real deltavi;
		
		/// Real and imaginary part of the current trough the capacitor [A]
		Real currr;
		Real curri;
		
		///Real and imaginary part of the DC equivalent current source [A] 
		Real cureqr;
		Real cureqi;
		
		Real mGcr;
		Real mGci;


	public:
		Capacitor() { };

		/// define capacitor name, conected nodes and capacitance
		Capacitor(std::string name, Int src, Int dest, Real capacitance);

		/// initializes variables detalvr, deltavi, currr, curri, cureqr and curreqi
		void init(Real om, Real dt);
		
		/// Stamps DC equivalent resistance to the conductance matrix
		void applySystemMatrixStamp(SystemModel& system);
		
		/// Stamps DC equivalent current source to the current vector
		void applyRightSideVectorStamp(SystemModel& system) { }
		
		/// calculates the value of the current source for one time step and apply it to the current vector
		void step(SystemModel& system, Real time);
		
		/// Recalculates variables detalvr, deltavi, currr and curri based on the simulation results of one time step
		void postStep(SystemModel& system);

		/// Return current from the previous step
		Complex getCurrent(SystemModel& system);
	};
}
#endif
