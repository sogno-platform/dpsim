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

TurbineGovernor::TurbineGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init)
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
	mTm = Tm_init;

}


Real TurbineGovernor::step(Real Om, Real OmRef, Real PmRef, Real dt) {

	// ### Governing ###
	// Input of speed relay
	Psr_in = PmRef + (OmRef - Om)*mK;
	// Input of servor motor
	Psm_in = Euler(Psm_in, -1, 1, dt / mTsr, Psr_in);
	// rate of change of valve
	mpVcv = (Psm_in - mVcv) / mTsm;
	if (mpVcv >= 0.1)
		mpVcv = 0.1;
	else if (mpVcv <= -1)
		mpVcv = -1;
	//Valve position
	mVcv = mVcv + dt*mpVcv;
	if (mVcv >= 1)
		mVcv = 1;
	else if (mVcv <= 0)
		mVcv = 0;
	//### Turbine ###
	// Simplified equation
	mTm = Euler(mTm, -1 / mTb, 1 / mTb, mpVcv*mFa, dt, mVcv);
	
	return mTm;

}



