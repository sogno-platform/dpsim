#ifndef SYNCHRONGENERATOR_H
#define SYNCHRONGENERATOR_H

#include "BaseComponent.h"
#include "ComponentCommons.h"

namespace DPsim {

	/// Synchronous generator model
	/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor 
	/// referred values. The calculation to per unit is performed in the initialization.
	/// The case where parInPerUnit is not set will be implemented later.
	/// parameter names include underscores and typical variables names found in literature instead of
	/// descriptive names in order to shorten formulas and increase the readability

	class SynchronGenerator : public BaseComponent {
	protected:

		// ### Machine parameters ###
		/// nominal power Pn [VA]
		Real mNomPower;
		/// nominal voltage Vn [V] (RMS)
		Real mNomVolt;
		/// nominal frequency fn [Hz]
		Real mNomFreq;
		/// nominal field current Ifn [A]
		Real mNomFieldCur;

		/// stator resistance Rs[Ohm]
		Real mRs;
		/// leakage inductance Ll [H]
		Real mLl;
		/// d-axis mutual inductance Lmd [H]
		Real mLmd;
		/// unsaturated d-axis mutual inductance Lmd [H]
		Real mLmd0;
		/// q-axis mutual inductance Lmq [H]
		Real mLmq;
		/// unsaturated q-axis mutual inductance Lmq [H]
		Real mLmq0;
		/// field resistance Rfd [Ohm]
		Real mRfd;
		/// field leakage inductance Llfd [H]
		Real mLlfd;
		/// d-axis damper resistance Rkd [Ohm]
		Real mRkd;
		/// d-axis damper leakage inductance Llkd [H]
		Real mLlkd;
		/// q-axis damper resistance 1 Rkq1 [Ohm]
		Real mRkq1;
		/// q-axis damper leakage inductance 1 Llkq1 [H]
		Real mLlkq1;
		/// q-axis damper resistance 2 Rkq2 [Ohm]
		Real mRkq2;
		/// q-axis damper leakage inductance 2 Llkq2 [H]
		Real mLlkq2;
		/// q winding inductance
		Real mLaq;
		/// d winding inductance
		Real mLad;

		/// inertia J [kg*m^2]
		Real mJ;
		/// number of poles
		int mPoleNumber;
		/// inertia coefficient H
		Real mH;

		// ### Stator base values ###
		/// specifies if the machine parameters are transformed to per unit
		SynchGenStateType mStateType;
		/// base stator voltage
		Real mBase_v;
		/// base stator voltage RMS
		Real mBase_V_RMS;
		/// base stator current
		Real mBase_i;
		/// base stator current RMS
		Real mBase_I_RMS;
		/// base stator impedance
		Real mBase_Z;
		/// base electrical angular frequency
		Real mBase_OmElec;
		/// base mechanical angular frequency
		Real mBase_OmMech;
		/// base stator inductance
		Real mBase_L;
		/// base torque
		Real mBase_T;
		/// base flux linkage
		Real mBase_Psi;

		/// base field current
		Real mBase_ifd;
		/// base field voltage
		Real mBase_vfd;
		/// base field impedance
		Real mBase_Zfd;
		/// base field inductance
		Real mBase_Lfd;


		// ### State variables ###
		/// rotor speed omega_r
		Real mOmMech;
		/// theta
		Real mThetaMech;
		/// mechanical Power Pm [W]
		Real mMechPower;
		/// mechanical torque
		Real mMechTorque;
		/// electrical torque
		Real mElecTorque;
		/// voltage vector q d 0 kq1 kq2 df kd
		DPSMatrix mVoltages = DPSMatrix::Zero(7, 1);
		/// flux linkage vector
		DPSMatrix mFluxes = DPSMatrix::Zero(7, 1);
		/// current vector
		DPSMatrix mCurrents = DPSMatrix::Zero(7, 1);
		/// interface voltage vector abcs
		DPSMatrix mAbcsVoltages = DPSMatrix::Zero(6, 1);
		/// interface current vector abcs
		DPSMatrix mAbcsCurrents = DPSMatrix::Zero(6, 1);
		/// interface voltage vector dq0
		DPSMatrix mDq0Voltages = DPSMatrix::Zero(3, 1);
		/// interface current vector dq0
		DPSMatrix mDq0Currents = DPSMatrix::Zero(3, 1);

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
		SynchronGenerator() { };
		
		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or 
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		SynchronGenerator(std::string name, int node1, int node2, int node3,
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
		/// Function parameters have to be given in Real units.
		void init(Real om, Real dt,
			Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle);

		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in Real units.
		void initStatesInPerUnit(Real initActivePower, Real initReactivePower,
			Real initTerminalVolt, Real initVoltAngle);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector.
		void step(SystemModel& system, Real fieldVoltage, Real mechPower);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower);

		/// Retrieves calculated voltage from simulation for next step
		void postStep(SystemModel& system);

		/// Park transform as described in Krause
		DPSMatrix abcToDq0Transform(Real theta, DPSMatrix& in);

		/// Inverse Park transform as described in Krause
		DPSMatrix dq0ToAbcTransform(Real theta, DPSMatrix& in);

		DPSMatrix getVoltages() { return mVoltages; }
		DPSMatrix getCurrents() { return mCurrents; }
		DPSMatrix getFluxes() { return mFluxes; }

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system) { }
	};
}
#endif
