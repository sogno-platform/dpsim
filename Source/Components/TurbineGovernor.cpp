/** Turbine Governor
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
*
* DPsim
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************************/

#include "TurbineGovernor.h"
#include "../IntegrationMethod.h"

using namespace DPsim;

Components::TurbineGovernor::TurbineGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm)
{
	mTa = Ta;
	mTb = Tb;
	mTc = Tc;
	mFa = Fa;
	mFb = Fb;
	mFc = Fc;
	mK = K;
	mTsr = Tsr;
	mTsm = Tsm;
}

void Components::TurbineGovernor::initialize(Real PmRef, Real Tm_init)
{
	mTm = Tm_init;
	mVcv = PmRef;
	mpVcv = 0;
	Psm_in = PmRef;
	T1 = (1 - mFa)*PmRef;
	T2 = mFa*PmRef;
}

Real Components::TurbineGovernor::step(Real Om, Real OmRef, Real PmRef, Real dt)
{
	// ### Governing ###
	// Input of speed relay
	Real Psr_in_hist = Psr_in;
	Psr_in = PmRef + (OmRef - Om)*mK;
	// Input of servor motor
	//Psm_in = Trapezoidal(Psm_in, -1, 1, dt / mTsr, Psr_in);
	Psm_in = Psm_in *(1 - dt / (2 * mTsr)) / (1 + dt / (2 * mTsr)) + (dt / 2) / (1 + dt / (2 * mTsr)) * (1 / mTsr)*(Psr_in + Psr_in_hist);
	// rate of change of valve
	Real mpVcv_hist = mpVcv;
	mpVcv = (Psm_in - mVcv) / mTsm;
	if (mpVcv >= 0.1)
		mpVcv = 0.1;
	else if (mpVcv <= -1)
		mpVcv = -1;
	//Valve position
	Real mVcv_hist = mVcv;
	mVcv = mVcv + dt/2*(mpVcv + mpVcv_hist);
	if (mVcv >= 1)
		mVcv = 1;
	else if (mVcv <= 0)
		mVcv = 0;

	//### Turbine ###
	// Simplified equation
	Real T1_hist = T1;
	//T1 = Trapezoidal(T1, -1, (1 - mFa), dt / mTb, mVcv);
	T1 = T1 *(1 - dt / (2*mTb)) / (1 + dt / (2*mTb)) + (dt / 2) / (1 + dt / (2 * mTb)) * ((1 - mFa) / mTb)*(mVcv + mVcv_hist);
	T2 = mVcv*mFa;
	mTm = T1 + T2;

	return mTm;
}
