#include "PiLine.h"

using namespace DPsim;

PiLine::PiLine(std::string name, int node1, int node2, int node3, Real resistance, Real inductance, Real capacitance) : BaseComponent(name, node1, node2, node3) {
	mResistance = resistance;
	mConductance = 1.0 / resistance;
	mInductance = inductance;
	mCapacitance = capacitance / 2;
}

void PiLine::applySystemMatrixStamp(SystemModel& system) {

	Real a = system.getTimeStep() / (2. * mInductance);
	Real b = system.getTimeStep() * system.getOmega() / 2.;
	mGlr = a / (1 + b*b);
	mGli = -a*b / (1 + b*b);
	mPrevCurFacRe = (1 - b*b) / (1 + b*b);
	mPrevCurFacIm = -2. * b / (1 + b*b);

	// Resistive part
	// Set diagonal entries
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mConductance, 0);
	}
	if (mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode3, mConductance, 0);
	}
	// Set off diagonal entries
	if (mNode1 >= 0 && mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode3, -mConductance, 0);
		system.addCompToSystemMatrix(mNode3, mNode1, -mConductance, 0);
	}

	// Inductance part
	// Set diagonal entries
	if (mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode3, mGlr, mGli);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGlr, mGli);
	}

	if (mNode3 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode2, -mGlr, -mGli);
		system.addCompToSystemMatrix(mNode2, mNode3, -mGlr, -mGli);
	}

	//capacitive part
	mGcr = 2.0 * mCapacitance / system.getTimeStep();
	mGci = system.getOmega() * mCapacitance;

	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mGcr, mGci);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGcr, mGci);
	}

}

void PiLine::init(Real om, Real dt) {
	// Initialize internal state
	mCurrIndRe = 0;
	mCurrIndIm = 0;

	mCurrCapRe1 = 0;
	mCurrCapIm1 = 0;

	mCurrCapRe2 = 0;
	mCurrCapIm2 = 0;

	mCurEqIndRe = 0;
	mCurEqIndIm = 0;

	mCurEqCapRe1 = 0;
	mCurEqCapIm1 = 0;

	mCurEqCapRe2 = 0;
	mCurEqCapIm2 = 0;

	mDeltaVre = 0;
	mDeltaVim = 0;

	mVoltageAtNode1Re = 0;
	mVoltageAtNode1Im = 0;

	mVoltageAtNode2Re = 0;
	mVoltageAtNode2Im = 0;
}

void PiLine::step(SystemModel& system, Real time) {

	// Initialize internal state inductance
	mCurEqIndRe = mGlr * mDeltaVre - mGli * mDeltaVim + mPrevCurFacRe * mCurrIndRe - mPrevCurFacIm * mCurrIndIm;
	mCurEqIndIm = mGli * mDeltaVre + mGlr * mDeltaVim + mPrevCurFacIm * mCurrIndRe + mPrevCurFacRe * mCurrIndIm;

	if (mNode3 >= 0) {
		system.addCompToRightSideVector(mNode3, -mCurEqIndRe, -mCurEqIndIm);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurEqIndRe, mCurEqIndIm);
	}

	// Initialize internal state capacitance 1

	mCurEqCapRe1 = mCurrCapRe1 + mGcr * mVoltageAtNode1Re + mGci * mVoltageAtNode1Im;
	mCurEqCapIm1 = mCurrCapIm1 + mGcr * mVoltageAtNode1Im - mGci * mVoltageAtNode1Re;

	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurEqCapRe1, mCurEqCapIm1);
	}

	// Initialize internal state capacitance 2
	mCurEqCapRe2 = mCurrCapRe2 + mGcr * mVoltageAtNode2Re + mGci * mVoltageAtNode2Im;
	mCurEqCapIm2 = mCurrCapIm2 + mGcr * mVoltageAtNode2Im - mGci * mVoltageAtNode2Re;
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurEqCapRe2, mCurEqCapIm2);
	}
}

void PiLine::postStep(SystemModel& system) {
	Real vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode3 >= 0) {
		system.getRealFromLeftSideVector(mNode3);
		vposr = system.getRealFromLeftSideVector(mNode3);
		vposi = system.getImagFromLeftSideVector(mNode3);
	}

	if (mNode2 >= 0) {
		system.getRealFromLeftSideVector(mNode2);
		vnegr = system.getRealFromLeftSideVector(mNode2);
		vnegi = system.getImagFromLeftSideVector(mNode2);
	}

	mDeltaVre = vposr - vnegr;
	mDeltaVim = vposi - vnegi;
	mCurrIndRe = mGlr * mDeltaVre - mGli * mDeltaVim + mCurEqIndRe;
	mCurrIndIm = mGli * mDeltaVre + mGlr * mDeltaVim + mCurEqIndIm;


	// extract solution
	if (mNode1 >= 0) {
		mVoltageAtNode1Re = system.getRealFromLeftSideVector(mNode1);
		mVoltageAtNode1Im = system.getImagFromLeftSideVector(mNode1);
	}

	if (mNode2 >= 0) {
		mVoltageAtNode2Re = system.getRealFromLeftSideVector(mNode2);
		mVoltageAtNode2Im = system.getImagFromLeftSideVector(mNode2);
	}


	mCurrCapRe1 = mGcr * mVoltageAtNode1Re - mGci * mVoltageAtNode1Im - mCurEqCapRe1;
	mCurrCapIm1 = mGci * mVoltageAtNode1Re + mGcr * mVoltageAtNode1Im - mCurEqCapIm1;

	mCurrCapRe2 = mGcr * mVoltageAtNode2Re - mGci * mVoltageAtNode2Im - mCurEqCapRe2;
	mCurrCapIm2 = mGci * mVoltageAtNode2Re + mGcr * mVoltageAtNode2Im - mCurEqCapIm2;

}
