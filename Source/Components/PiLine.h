#ifndef PILINE_H
#define PILINE_H

#include "../Components.h"

namespace DPsim {

	class PiLine : public BaseComponent {
	protected:
		///resistance in [ohms]
		Real mResistance;

		///Conductance of the line in [S]
		Real mConductance;

		///Capacitance of the line in [F]
		Real mCapacitance;

		///Inductance of the line in [H]
		Real mInductance;

		/// Real part of voltage at node 1 [V]
		Real mVoltageAtNode1Re;
		/// Imaginary part of voltage at node 1 [V]
		Real mVoltageAtNode1Im;
		/// Real part of voltage at node 2 [V]
		Real mVoltageAtNode2Re;
		/// Imaginary part of voltage at node 2 [V]
		Real mVoltageAtNode2Im;
		/// Real part of voltage across the inductor [V]
		Real mDeltaVre;
		/// Imaginary part of voltage across the inductor [V]
		Real mDeltaVim;

		/// Real part of inductor current [A]
		Real mCurrIndRe;
		/// Imaginary part of inductor current [A]
		Real mCurrIndIm;
		/// Real part of capacitor1 current [A]
		Real mCurrCapRe1;
		/// Imaginary part of capacitor1 current [A]
		Real mCurrCapIm1;
		/// Real part of capacitor2 current [A]
		Real mCurrCapRe2;
		/// Imaginary part of capacitor2 current [A]
		Real mCurrCapIm2;

		/// Real part of inductor equivalent current source [A]
		Real mCurEqIndRe;
		/// Imaginary part of inductor equivalent current source [A]
		Real mCurEqIndIm;
		/// Real part of capacitor1 equivalent current source [A]
		Real mCurEqCapRe1;
		/// Imaginary part of capacitor1 equivalent current source [A]
		Real mCurEqCapIm1;
		/// Real part of capacitor2 equivalent current source [A]
		Real mCurEqCapRe2;
		/// Imaginary part of capacitor2 equivalent current source [A]
		Real mCurEqCapIm2;

		/// Real part of inductor equivalent conductance [S]
		Real mGlr;
		/// Imaginary part of inductor equivalent conductance [S]
		Real mGli;
		/// Real part of capacitor equivalent conductance [S]
		Real mGcr;
		/// Imaginary part of capacitor equivalent conductance [S]
		Real mGci;

		/// Auxiliar variable
		Real mPrevCurFacRe;
		/// Auxiliar variable
		Real mPrevCurFacIm;

	public:
		PiLine() { };

		/// Define line name, conected nodes, resistance, inductance and capacitance 
		PiLine(std::string name, int node1, int node2, int node3, Real resistance, Real inductance, Real capacitance);

		/// Initialize voltages and currents values
		void init(Real om, Real dt);

		/// Stamps resistances to the conductance matrix
		void applySystemMatrixStamp(SystemModel& system);

		/// does nothing
		void applyRightSideVectorStamp(SystemModel& system) { }

		/// calculates the value of the DC equivalent current sources for one time step and apply matrix stamp to the current vector
		void step(SystemModel& system, Real time);

		/// Recalculates voltages and currents based on the simulation results of one time step
		void postStep(SystemModel& system);
	};
}

#endif
