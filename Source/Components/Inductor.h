#ifndef INDUCTOR_H
#define INDUCTOR_H

#include "BaseComponent.h"


namespace DPsim {

	/// Inductor model:
	/// The inductor is represented by a DC equivalent circuit which corresponds to one iteration of the trapezoidal integration method.
	/// The equivalent DC circuit is a resistance in paralel with a current source. The resistance is constant for a defined time step and system
	///frequency and the current source changes for each iteration.
	class Inductor : public BaseComponent {
	protected:
	
		/// Inductance [H]
		Real mInductance;
		
		/// Real and imaginary part of the voltage across the inductor [V]
		Real mDeltaVre;
		Real mDeltaVim;
		
		/// Real and imaginary part of the current trough the inductor [A]
		Real mCurrRe;
		Real mCurrIm;
		
		/// Real and imaginary part of the DC equivalent current source [A]
		Real mCurEqRe;
		Real mCurEqIm;
		
		/// Real and imaginary part of the DC equivalent conductance [S]
		Real mGlr;
		Real mGli;
		
		/// Auxiliar variables
		Real mPrevCurFacRe;
		Real mPrevCurFacIm;

	public:
		Inductor() { };
		
		/// Define inductor name, conected nodes and inductance 
		Inductor(std::string name, int src, int dest, double inductance);

		/// Initializes variables detalvr, deltavi, currr, curri, cureqr and curreqi
		void init(Real om, Real dt);
		
		/// Stamps DC equivalent resistance to the conductance matrix
		void applySystemMatrixStamp(SystemModel& system);
		
		/// Stamps DC equivalent current source to the current vector
		void applyRightSideVectorStamp(SystemModel& system) { }
		
		/// calculates the value of the current source for one time step and apply it to the current vector
		void step(SystemModel& system, Real time);
		
		/// Recalculates variables detalvr, deltavi, currr and curri based on the simulation results of one time step
		void postStep(SystemModel& system);
	};
}
#endif