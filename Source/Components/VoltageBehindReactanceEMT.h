#ifndef VOLTAGEBEHINDREACTANCEEMT_H
#define VOLTAGEBEHINDREACTANCE_H

#include "BaseComponent.h"
#include "ComponentCommons.h"

namespace DPsim {

	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor 
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability

	class VoltageBehindReactanceEMT : public BaseComponent {
	protected:

		// ### Machine parameters ###
		/// nominal power Pn [VA]
		double mNomPower;
		/// nominal voltage Vn [V] (RMS)
		double mNomVolt;
		/// nominal frequency fn [Hz]
		double mNomFreq;
		/// nominal field current Ifn [A]
		double mNomFieldCur;

		/// stator resistance Rs[Ohm]
		double mRs;
		/// leakage inductance Ll [H]
		double mLl;
		/// d-axis mutual inductance Lmd [H]
		double mLmd;
		/// unsaturated d-axis mutual inductance Lmd [H]
		double mLmd0;
		/// q-axis mutual inductance Lmq [H]
		double mLmq;
		/// unsaturated q-axis mutual inductance Lmq [H]
		double mLmq0;
		/// field resistance Rfd [Ohm]
		double mRfd;
		/// field leakage inductance Llfd [H]
		double mLlfd;
		/// d-axis damper resistance Rkd [Ohm]
		double mRkd;
		/// d-axis damper leakage inductance Llkd [H]
		double mLlkd;
		/// q-axis damper resistance 1 Rkq1 [Ohm]
		double mRkq1;
		/// q-axis damper leakage inductance 1 Llkq1 [H]
		double mLlkq1;
		/// q-axis damper resistance 2 Rkq2 [Ohm]
		double mRkq2;
		/// q-axis damper leakage inductance 2 Llkq2 [H]
		double mLlkq2;
		/// q winding inductance
		double mLaq;
		/// d winding inductance
		double mLad;

		/// d dynamic inductance
		double mDLmd;
		/// q dynamic inductance
		double mDLmq;

		/// d dynamic flux
		double mDPsid;
		/// q dynamic flux
		double mDPsiq;

		/// Dynamic d voltage
		double mDVq;
		/// Dynamic q voltage
		double mDVd;

		/// Dynamic voltage phase a
		double mDVa;
		/// Dynamic voltage phase b
		double mDVb;
		/// Dynamic voltage phase c
		double mDVc;

		//Theta mech hist
		double mThetaMech_hist1;
		double mThetaMech_hist2;

		//Theta omega hist
		double mOmMech_hist;
		double mOmMech_hist1;
		double mOmMech_hist2;

		/// inertia J [kg*m^2]
		double mJ;
		/// number of poles
		int mPoleNumber;
		/// inertia coefficient H
		double mH;

		// ### Stator base values ###
		/// specifies if the machine parameters are transformed to per unit
		SynchGenStateType mStateType;
		/// base stator voltage
		double mBase_v;
		/// base stator voltage RMS
		double mBase_V_RMS;
		/// base stator current
		double mBase_i;
		/// base stator current RMS
		double mBase_I_RMS;
		/// base stator impedance
		double mBase_Z;
		/// base electrical angular frequency
		double mBase_OmElec;
		/// base mechanical angular frequency
		double mBase_OmMech;
		/// base stator inductance
		double mBase_L;
		/// base torque
		double mBase_T;
		/// base flux linkage
		double mBase_Psi;

		/// base field current
		double mBase_ifd;
		/// base field voltage
		double mBase_vfd;
		/// base field impedance
		double mBase_Zfd;
		/// base field inductance
		double mBase_Lfd;


		// ### State variables ###
		/// rotor speed omega_r
		double mOmMech;
		/// theta
		double mThetaMech;
		double mThetaMech2;
		/// mechanical Power Pm [W]
		double mMechPower;
		/// mechanical torque
		double mMechTorque;
		/// electrical torque
		double mElecTorque;
		double mElecTorque_hist;



		/// voltage vector q d 0 kq1 kq2 df kd
		DPSMatrix mVoltages2 = DPSMatrix::Zero(7, 1);
		/// flux linkage vector
		DPSMatrix mFluxes = DPSMatrix::Zero(7, 1);
		/// current vector
		DPSMatrix mCurrents2 = DPSMatrix::Zero(7, 1);

		/// voltage vector q d 0 fd kd kq1 kq2
		double mVd;
		double mVq;
		double mV0;
		double mVfd;
		double mVkd;
		double mVkq1;
		double mVkq2;

		/// current vector q d 0 fd kd kq1 kq2
		double mId;
		double mIq;
		double mI0;
		double mIfd;
		double mIkd;
		double mIkq1;
		double mIkq2;

		/// flux linkage vector q d 0 fd kd kq1 kq2
		double mPsid;
		double mPsiq;
		double mPsi0;
		double mPsifd;
		double mPsikd;
		double mPsikq1;
		double mPsikq2;

		/// Interface voltage vector
		double mVa;
		double mVb;
		double mVc;

		/// Interface voltage vector
		double mIa;
		double mIb;
		double mIc;

		/// Magnetizing flux linkage in qd axes
		Real mPsimq;
		Real mPsimd;

		/// Auxiliar variables
		Real b11;
		Real b12;
		Real b13;
		Real b21;
		Real b22;
		Real b23;
		Real b31;
		Real b32;
		Real b33;
		Real b41;
		Real b42;
		Real b43;
		Real c11;
		Real c12;
		Real c23;
		Real c24;
		Real c15;
		Real c25;
		Real c26;
		Real c21_omega;
		Real c22_omega;
		Real c13_omega;
		Real c14_omega;

		/// Auxiliar matrix
		DPSMatrix E1 = DPSMatrix::Zero(2, 2);
		DPSMatrix Ea = DPSMatrix::Zero(2, 2);
		DPSMatrix E1b = DPSMatrix::Zero(2, 1);
		DPSMatrix E2 = DPSMatrix::Zero(2, 2);
		DPSMatrix E2b = DPSMatrix::Zero(2, 2);
		DPSMatrix F1 = DPSMatrix::Zero(2, 2);
		DPSMatrix Fa = DPSMatrix::Zero(2, 2);
		DPSMatrix F1b = DPSMatrix::Zero(2, 1);
		DPSMatrix F2 = DPSMatrix::Zero(2, 2);
		DPSMatrix F2b = DPSMatrix::Zero(2, 2);
		DPSMatrix F3 = DPSMatrix::Zero(2, 2);
		DPSMatrix F3b = DPSMatrix::Zero(2, 1);
		DPSMatrix K1a = DPSMatrix::Zero(2, 2);
		DPSMatrix K1b = DPSMatrix::Zero(2, 1);
		DPSMatrix K1;
		DPSMatrix K2a = DPSMatrix::Zero(2, 2);
		DPSMatrix K2b = DPSMatrix::Zero(2, 1);
		DPSMatrix K2;
		DPSMatrix ParkMat = DPSMatrix::Zero(3, 3);
		DPSMatrix InverseParkMat = DPSMatrix::Zero(3, 3);
		DPSMatrix K = DPSMatrix::Zero(3, 3);
		DPSMatrix C26 = DPSMatrix::Zero(2, 1);
		DPSMatrix H_qdr;

		/// Equivalent voltage behind reactance resistance matrix
		DPSMatrix R_vbr_eq = DPSMatrix::Zero(3, 3);

		/// Equivalent voltage behind reactance rotor voltage vector
		DPSMatrix e_r_vbr = DPSMatrix::Zero(3, 1);

		/// Equivalent voltage behind reactance voltage vector
		DPSMatrix e_h_vbr;

		/// Flux vector
		DPSMatrix mPsikq1kq2 = DPSMatrix::Zero(2, 1);
		DPSMatrix mPsifdkd = DPSMatrix::Zero(2, 1);


		// ### Useful Matrices ###
		/// inductance matrix
		DPSMatrix mDInductanceMat = DPSMatrix::Zero(3, 3);
		/// resistance matrix
		DPSMatrix mResistanceMat = DPSMatrix::Zero(3, 3);
		/// reactance matrix
		//DPSMatrix mReactanceMat = DPSMatrix::Zero(7, 7);
		/// omega - flux matrix
		//DPSMatrix mOmegaFluxMat = DPSMatrix::Zero(7, 7);
		/// matrix for reversing stator current directions in calculations with respect to other currents
		//DPSMatrix mReverseCurrents = DPSMatrix::Zero(7, 7);

		double mLa;
		double mLb;

		//Historical term of voltage
		DPSMatrix mV_hist = DPSMatrix::Zero(3, 1);

		//Phase Voltages in pu
		DPSMatrix mVabc = DPSMatrix::Zero(3, 1);

		//Historical term of current
		DPSMatrix mIabc = DPSMatrix::Zero(3, 1);

		//Historical term of voltage
		DPSMatrix mDVabc = DPSMatrix::Zero(3, 1);

		//Phase Voltages
		DPSMatrix mVoltageVector = DPSMatrix::Zero(3, 1);

		//Phase Currents
		DPSMatrix mCurrentVector = DPSMatrix::Zero(3, 1);


	public:
		VoltageBehindReactanceEMT() { };

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or 
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		VoltageBehindReactanceEMT(std::string name, int node1, int node2, int node3,
			Real nomPower, Real nomVolt, Real nomFreq, int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia);

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit.
		/// The initialization mode depends on the setting of state type.
		void initWithPerUnitParam(
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd, Real Rkq1, Real Llkq1,
			Real Rkq2, Real Llkq2,
			Real H);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type. 
		/// Function parameters have to be given in real units.
		void init(Real om, Real dt,
			Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle);

		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void initStatesInPerUnit(Real initActivePower, Real initReactivePower,
			Real initTerminalVolt, Real initVoltAngle);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector.
		void step(SystemModel& system, Real fieldVoltage, Real mechPower, Real time);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower, Real time, NumericalMethod numMethod);

		void FormTheveninEquivalent(Real dt);

		/// Retrieves calculated voltage from simulation for next step
		void postStep(SystemModel& system);

		/// Park transform as described in Krause
		DPSMatrix parkTransform(Real theta, double a, double b, double c);
		//DPSMatrix parkTransform(Real theta, DPSMatrix& in);

		/// Inverse Park transform as described in Krause
		DPSMatrix inverseParkTransform(Real theta, double q, double d, double zero);
		//DPSMatrix inverseParkTransform(Real theta, DPSMatrix& in);

		DPSMatrix& getVoltages() { return mVoltageVector; }
		DPSMatrix& getCurrents() { return mCurrentVector; }
		//DPSMatrix& getFluxes() { return mFluxes; }
		double getElectricalTorque() { return mElecTorque; }
		double getRotationalSpeed() { return mOmMech; }
		double getRotorPosition() { return mThetaMech; }

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time) { }
	};
}
#endif
