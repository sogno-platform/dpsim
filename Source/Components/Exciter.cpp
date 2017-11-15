/** Exciter
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
* @license GNU General Public License (version 3)
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

Exciter::Exciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd, Real Vf_init)
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
	mVf = Vf_init*(Lad / Rfd);

}

Exciter::~Exciter() {

}



//void Exciter::init(Real mVd, Real mVq, Real Vfd, Real Vref, Real Lad, Real Rfd) {
//
//
//	mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
//	mVf = (Lad / Rfd)*Vfd;
//	mVref = Vref;
//	
//}


void Exciter::step(Real mVd, Real mVq, Real Vref, Real dt, Real time) {


	mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
	// Voltage Transducer equation
	mVm = Euler(mVm, -1, 1, dt / mTr, mVh);
	// Stabilizing feedback equation
	Real dUf = (mVr - mVse - mKe*mVf) / mTe;
	mVis = Euler(mVis, -1, mKf, dt / mTf, dUf);
	// Amplifier equation
	mVr = Euler(mVr, -1, mKa, dt / mTa, mVref - mVm - mVis);
	// Exciter
	mVse = 0.0039*exp(mVf*1.555);
	mVf = Euler(mVf, -mKe, 1, dt / mTe, mVr - mVse);

	mVfd = (mRfd / mLad)*mVf;

}


//void Exciter::stepInPerUnit(Real mVd, Real mVq, Real Vref, Real dt) {
//
//
//
//}


//void Exciter::postStep(SystemModel& system) {
//
//}



