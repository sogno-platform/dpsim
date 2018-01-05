/** Exciter
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

#include "Exciter.h"
#include "../IntegrationMethod.h"

using namespace DPsim;

Exciter::Exciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd)
{

	mTa = Ta;
	mKa = Ka;
	mTe = Te;
	mKe = Ke;
	mTf = Tf;
	mKf = Kf;
	mTr = Tr;
	mLad = Lad;
	mRfd = Rfd;

}

void Exciter::init(Real mVd, Real mVq, Real Vref, Real Vf_init) {

		//Field voltage
		mVf = Vf_init*(mLad / mRfd);
		//mVf = 1;
		mVse = 0.0039*exp(mVf*1.555)*mVf;
		mVr = mVse + mKe*mVf;
		mVf_init = mVr/mKa;
		//mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
		mVh = 1;
		mVm = mVh;
		mVis = 0;

}

Real Exciter::step(Real mVd, Real mVq, Real Vref, Real dt) {

	mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
	// Voltage Transducer equation
	mVm = Euler(mVm, -1, 1, dt / mTr, mVh);
	// Stabilizing feedback equation
	//mVis = (mKf / mTf)*mVf - mVfl;
	//mVfl = mVfl + dt*mVis / mTf;
	mVis = Euler(mVis, -1, mKf, dt / mTf, ((mVr - mVse) - mVf*mKe)/mTe);
	// Amplifier equation
	mVr = Euler(mVr, -1, mKa, dt / mTa, Vref - mVm - mVis + mVf_init);
	// Exciter
	mVse = 0.0039*exp(mVf*1.555)*mVf;
	mVf = Euler(mVf, -mKe, 1, dt / mTe, mVr - mVse);

	return (mRfd / mLad)*mVf;

}



