#include <dpsim-models/Signal/VSIControlType1.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::VSIControlType1::VSIControlType1(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { }

void VSIControlType1::setParameters(std::shared_ptr<Base::TurbineParameters> parameters) {
    if (auto params = std::dynamic_pointer_cast<Signal::VSIControlType1Parameters>(parameters)){
        mParameters = params;
        SPDLOG_LOGGER_INFO(mSLog, 
			"\nVSIController Type1 Parameters: "
			"\nKpv: {:e}"
			"\nKiv: {:e}"
			"\nKpc: {:e}"
			"\nKic: {:e}",
			mParameters->Kpv, mParameters->Kiv,
			mParameters->Kpc, mParameters->Kic);
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be VSIControlType1!" << std::endl;
		throw CPS::TypeException();
	}
}

void VSIControlType1::initialize(Complex Vsref, Idq_filter) {
    Complex Vsref_dq = Math::rotatingFrame2to1((**mVsref)(0,0), **mThetaInv, **mThetaSys);
	mPhi_d = **mIfilter_d / mKiv;
	mPhi_q = **mIfilter_q / mKiv; 
	mGamma_d = (Vsref_dq.real() - **mVcap_d) / mKic;
	mGamma_q = (Vsref_dq.imag() - **mVcap_q) / mKic;

	SPDLOG_LOGGER_INFO(mLogger,
			"\nInitialize controller states:"	  
			"\n\tPhi_d = {}"
			"\n\tPhi_q = {}"
			"\n\tGamma_d = {}"
			"\n\tGamma_q = {}",
			mPhi_d, mPhi_q, mGamma_d, mGamma_q);
	mLogger->flush();

    // initialize state matrix
    // [x] = [phid, phiq, gammad, gammaq]
	// [u] = [vdref, vqref, vdc, vqc, idc, idq]
	// [y] = [vdout, vqout]

    mA <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		mParameters->Kiv, 0, 0, 0,
		0, mParameters->Kiv, 0, 0;

	mB <<
		1, 0, -1, 0, 0, 0,
		0, 1, 0, -1, 0, 0,
		mParameters->Kpv, 0, -mParameters->Kpv, 0, -1, 0,
		0, mParameters->Kpv, 0, -mParameters->Kpv, 0, -1;

	mC <<
		mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic, 0,
		0, mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic;

	mD <<
		mParameters->Kpc * mParameters->Kpv , 0, -mParameters->Kpc * mParameters->Kpv + 1, 0, -mParameters->Kpc, 0,
		0, mParameters->Kpc * mParameters->Kpv, 0, -mParameters->Kpc * mParameters->Kpv + 1, 0, -mParameters->Kpc;

    Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, Matri::Zero(4,1), mTimeStep, mATrapezoidal, mBTrapezoidal, mCTrapezoidal);

    // Log state-space matrices
	SPDLOG_LOGGER_INFO(mSLog, "State space matrices:");
    SPDLOG_LOGGER_INFO(mSLog, "A = \n{}", mA);
    SPDLOG_LOGGER_INFO(mSLog, "B = \n{}", mB);
    SPDLOG_LOGGER_INFO(mSLog, "C = \n{}", mC);
    SPDLOG_LOGGER_INFO(mSLog, "D = \n{}", mD);
    mLogger->flush();
}

Real VSIControlType1::step(Real Vcap_d, Real Vcap_q, Real Ifilter_d, Real Ifilter_q) {
    
    // get current inputs
	mInputCurr << mParameters->VdRef, mParameters->VqRef, Vcap_d, Vcap_q, Ifilter_d, Ifilter_q;
    SPDLOG_LOGGER_DEBUG(mSLog, "Time {}\n: inputCurr = \n{}\n , inputPrev = \n{}\n , statePrev = \n{}", time, mInputCurr, mInputPrev, mStatePrev);

	// calculate new states
	//**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
    **mStateCurr = Math::applyStateSpaceTrapezoidalMatrices(mATrapezoidal, mBTrapezoidal, mCTrapezoidal, **mStatePrev, **mInputCurr, **mInputPrev);
	SPDLOG_LOGGER_DEBUG(mSLog, "stateCurr = \n {}", **mStateCurr);

	// calculate new outputs
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	SPDLOG_LOGGER_DEBUG(mSLog, "Output values: outputCurr = \n{}", **mOutputCurr);
    
    //TODO: RETURN?
}

