#ifndef SYNCHRONGENERATOREMT_H
#define SYNCHRONGENERATOREMT_H

#include "BaseComponent.h"
#include "ComponentCommons.h"

namespace DPsim {

	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor 
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability

	class SynchronGeneratorEMT : public BaseComponent {
	protected:

		Logger* mLog;

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

		/// Determinant of Ld (inductance matrix of d axis)
		double detLd;
		/// Determinant of Lq (inductance matrix of q axis)
		double detLq;

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

		/// mechanical torque
		double mMechTorque_past;
		/// electrical torque
		double mElecTorque_past;
		/// rotor speed omega_r
		double mOmMech_past;

		/// voltage vector q d 0 kq1 kq2 df kd
		DPSMatrix mVoltages2 = DPSMatrix::Zero(7, 1);
		/// flux linkage vector
		DPSMatrix mFluxes2 = DPSMatrix::Zero(7, 1);
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

		double mId_past;
		double mIq_past;

		/// flux linkage vector q d 0 fd kd kq1 kq2
		double mPsid;
		double mPsiq;
		double mPsi0;
		double mPsifd;
		double mPsikd;
		double mPsikq1;
		double mPsikq2;

		double mPsid_past;
		double mPsiq_past;

		/// Interface voltage vector
		double mVa;
		double mVb;
		double mVc;

		/// Interface voltage vector
		double mIa;
		double mIb;
		double mIc;

		/// Number of damping windings in q 
		int DampingWindings;

		// ### Useful Matrices ###
		/// inductance matrix
		DPSMatrix mInductanceMat = DPSMatrix::Zero(7, 7);
		/// resistance matrix
		DPSMatrix mResistanceMat = DPSMatrix::Zero(7, 7);
		/// reactance matrix
		DPSMatrix mReactanceMat = DPSMatrix::Zero(7, 7);
		/// omega - flux matrix
		DPSMatrix mOmegaFluxMat = DPSMatrix::Zero(7, 7);
		/// matrix for reversing stator current directions in calculations with respect to other currents
		DPSMatrix mReverseCurrents = DPSMatrix::Zero(7, 7);


	public:
		SynchronGeneratorEMT() { };
		~SynchronGeneratorEMT();

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or 
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		SynchronGeneratorEMT(std::string name, int node1, int node2, int node3,
			Real nomPower, Real nomVolt, Real nomFreq, int poleNumber, Real nomFieldCur,
			Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
			Real Rfd, Real Llfd, Real Rkd, Real Llkd,
			Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
			Real inertia, bool logActive = false);

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
			Real initActivePower, Real initReactivePower, Real initTerminalVolt, 
			Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void initStatesInPerUnit(Real initActivePower, Real initReactivePower, Real initTerminalVolt,
			Real initVoltAngle, Real initFieldVoltage, Real initMechPower);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector.
		void step(SystemModel& system, Real time);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod);

		/// Retrieves calculated voltage from simulation for next step
		void postStep(SystemModel& system);

		/// Park transform as described in Krause
		//DPSMatrix parkTransform(Real theta, DPSMatrix& in);
		DPSMatrix parkTransform2(Real theta, double a, double b, double c);

		/// Inverse Park transform as described in Krause
		//DPSMatrix inverseParkTransform(Real theta, DPSMatrix& in);
		DPSMatrix inverseParkTransform2(Real theta, double d, double q, double zero);

		DPSMatrix& getVoltages() { return mVoltages2; }
		DPSMatrix& getCurrents() { return mCurrents2; }
		DPSMatrix& getFluxes() { return mFluxes2; }

		// Methods for network integrated components
		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system) { }
	};
}
#endif
