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

Components::Exciter::Exciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd)
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


void Components::Exciter::initialize(Real Vh_init, Real Efd_init)
{
	mEfd = Efd_init;
	mVx = mEfd <= 2.3 ? 0.1 / 2.3 : 0.33 / 3.1;
	mVx *= mEfd;

	mVr = mVx + mKe*mEfd;
	mVt = Vh_init;
	mVc = mVt;
	mVf = 0;
}

Real Components::Exciter::step(Real mVd, Real mVq, Real Vref, Real dt)
{
	mVt = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
	// Voltage Transducer equation
	mVc = Trapezoidal(mVc, -1, 1, dt / mTr, mVt);
	// Stabilizing feedback equation
	mVf = Trapezoidal(mVf, -1, mKf, dt / mTf, ((mVr - mVx) - mEfd*mKe)/mTe);
	// Amplifier equation
	mVr = Trapezoidal(mVr, -1, mKa, dt / mTa, Vref - mVc - mVf);
	if (mVr > 1)
			mVr = 1;
	else if (mVr < -0.9)
			mVr = -0.9;
	// Exciter
	if (mEfd <= 2.3)
			mVx = (0.1 / 2.3)*mEfd;
	else
			mVx = (0.33 / 3.1)*mEfd;
	mVx = mVx*mEfd;
	mEfd = Trapezoidal(mEfd, -mKe, 1, dt / mTe, mVr - mVx);

	return (mRfd / mLad)*mEfd;
}
