/**
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/EMT/EMT_Ph3_SynchronGeneratorDQODE.h>

using namespace CPS;

EMT::Ph3::SynchronGeneratorDQODE::SynchronGeneratorDQODE(String uid, String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(uid, name, logLevel) {
}

EMT::Ph3::SynchronGeneratorDQODE::SynchronGeneratorDQODE(String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(name, name, logLevel) {
}

void EMT::Ph3::SynchronGeneratorDQODE::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mDim = mNumDampingWindings + 7;
	mOdePreState = Matrix::Zero(mDim, 1);
	mOdePostState = Matrix::Zero(mDim, 1);
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mMnaTasks.push_back(std::make_shared<ODEPreStep>(*this));
}

void EMT::Ph3::SynchronGeneratorDQODE::MnaPreStep::execute(Real time, Int timeStepCount) {
	// ODEPreStep and ODESolver.Solve guaranteed to be executed by scheduler
	mSynGen.odePostStep();
	mSynGen.mnaApplyRightSideVectorStamp(mSynGen.mRightVector);
}

void EMT::Ph3::SynchronGeneratorDQODE::odePreStep() {
	for(int i=0; i<mDim-2;i++)
		mOdePreState(i)=mPsisr(i, 0);

	mOdePreState(mDim-2)=mThetaMech;
	mOdePreState(mDim-1)=mOmMech;

	//copied from stepInPerUnit
	mVdq0 = abcToDq0Transform(mThetaMech, mIntfVoltage);
	mVdq0 = mVdq0 / mBase_V;
}

void EMT::Ph3::SynchronGeneratorDQODE::ODEPreStep::execute(Real time, Int timeStepCount) {
	mSynGen.odePreStep();
}

void EMT::Ph3::SynchronGeneratorDQODE::odePostStep() {
	for(int i=0; i<mDim-2;i++)
		mPsisr(i,0)=mOdePostState(i);

	mThetaMech=mOdePostState(mDim-2);
	mOmMech=mOdePostState(mDim-1);

	//copied from stepInPerUnit ; makes only sense to call this after write back
	mIsr = mFluxToCurrentMat * mPsisr;

	mIdq0(0, 0) = mIsr(0, 0);
	mIdq0(1, 0) = mIsr(3, 0);
	mIdq0(2, 0) = mIsr(6, 0);
	mIntfCurrent = mBase_I * dq0ToAbcTransform(mThetaMech, mIdq0);

	SPDLOG_LOGGER_DEBUG(mSLog, "\nCurrent: \n{:s}",
		Logger::matrixCompToString(mIntfCurrent));
}

// ODE-Class simulation state-space
void EMT::Ph3::SynchronGeneratorDQODE::odeStateSpace(double t, const double y[], double ydot[]) {
	if(mDim==9){//flux-order: ds, fd, kd, qs, kq1, (kq2,) 0s
		//multiply voltages with mBase_OmElec (similar to stepInPerUnit):
		ydot[0]=mBase_OmElec*(mVdq0(0,0)+y[8]*y[3]); //v_d+omega*lambda_q
		ydot[1]=mBase_OmElec*mVsr(1,0); //v_fd
		ydot[2]=0;
		ydot[3]=mBase_OmElec*(mVdq0(1,0)-y[8]*y[0]); //v_q-omega*lambda_d
		ydot[4]=0;
		ydot[5]=0;
		ydot[6]=mBase_OmElec*mVdq0(2,0);//v_0
	}else{
		ydot[0]=mBase_OmElec*(mVdq0(0,0)+y[8]*y[3]); //v_d+omega*lambda_q
		ydot[1]=mBase_OmElec*mVsr(1,0); //v_fd
		ydot[2]=0;
		ydot[3]=mBase_OmElec*(mVdq0(1,0)-y[8]*y[0]); //v_q-omega*lambda_d
		ydot[4]=0;
		ydot[5]=mBase_OmElec*mVdq0(2,0);//v_0
	}

	// Matrix Vector-Mult:
	for(int i=0;i<mDim-2;i++)
		for(int j=0;j<mDim-2;j++)
			ydot[i]+=mBase_OmElec*mFluxStateSpaceMat(i,j)*y[j];

	// Mechanical equations:

	// Compute theta with transformation (following stepInPerUnit):
	ydot[mDim-2]= y[mDim-1] * mBase_OmMech;

	/* Auxiliary variables to compute
	 * T_e= lambda_d * i_q - lambda_q * i_d
	 * needed for omega:  */
	realtype i_d=0,i_q=0;
	// Compute new currents (depending on updated fluxes)
	for(int i=0; i<mDim-2;i++){
		i_d+=mFluxToCurrentMat(0,i)*y[i];
		i_q+=mFluxToCurrentMat(3,i)*y[i];
	}

	// Compute Omega (according to stepInPerUnit)
	ydot[mDim-1]=1/(2*mInertia)*(mMechTorque-(y[3]*i_d-y[0]*i_q));
}

void EMT::Ph3::SynchronGeneratorDQODE::odeJacobian(double t, const double y[], double fy[], double J[], double tmp1[], double tmp2[], double tmp3[]){
	// Fill Jacobian:
	// Fluxes:
	for(int i=0; i<mDim-2; i++)
		for(int j=0;j<mDim-2; j++)
			J[i*(mDim-2)+j]=mBase_OmElec*mFluxStateSpaceMat(i,j); //Note: J is row-major ordered

	//omega in flux eqs.:
	J[0*(mDim-2)+mDim-1]=mBase_OmElec*y[3];
	J[3*(mDim-2)+mDim-1]=mBase_OmElec*(-1)*y[0];
	//theta-row:
	J[(mDim-2)*(mDim-2)+mDim-1]=1*mBase_OmMech;

	realtype i_d=0,i_q=0;

	for(int i=0; i<mDim-2;i++){
		i_d+=mFluxToCurrentMat(0,i)*y[i];
		i_q+=mFluxToCurrentMat(3,i)*y[i];
	}
	//fluxes show up in omega eq. (with mBase_OmMech ?)
	J[(mDim-1)*(mDim-2)+0]=1/(2*mInertia)*i_q;
	J[(mDim-1)*(mDim-2)+3]=-1/(2*mInertia)*i_d;
}
