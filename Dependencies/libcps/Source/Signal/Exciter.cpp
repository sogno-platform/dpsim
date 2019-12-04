/** Exciter
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <cps/Signal/Exciter.h>
#include <cps/MathUtils.h>

using namespace CPS;

void Signal::Exciter::setParameters(Real Ta, Real Ka, Real Te, Real Ke, 
	Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd) {
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

void Signal::Exciter::initialize(Real Vh_init, Real Vf_init) {
	mVf = 1;
	mVse = mVf <= 2.3 ? 0.1 / 2.3 : 0.33 / 3.1;
	mVse *= mVf;

	mVr = mVse + mKe*mVf;
	mVf_init = mVr/mKa;
	mVh = 1;
	mVm = mVh;
	mVis = 0;
}

Real Signal::Exciter::step(Real mVd, Real mVq, Real Vref, Real dt) {
	mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
	// Voltage Transducer equation
	mVm = Math::StateSpaceEuler(mVm, -1, 1, dt / mTr, mVh);
	// Stabilizing feedback equation
	mVis = Math::StateSpaceEuler(mVis, -1, mKf, dt / mTf, ((mVr - mVse) - mVf*mKe)/mTe);
	// Amplifier equation
	mVr = Math::StateSpaceEuler(mVr, -1, mKa, dt / mTa, Vref - mVm - mVis + mVf_init);
	if (mVr > 1)
			mVr = 1;
	else if (mVr < -0.9)
			mVr = -0.9;
	// Exciter
	if (mVf <= 2.3)
			mVse = (0.1 / 2.3)*mVf;
	else
			mVse = (0.33 / 3.1)*mVf;
	mVse = mVse*mVf;
	mVf = Math::StateSpaceEuler(mVf, -mKe, 1, dt / mTe, mVr - mVse);

	return (mRfd / mLad)*mVf;
}
