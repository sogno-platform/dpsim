#include <dpsim-models/Signal/ExciterStatic.h>

using namespace CPS;
using namespace CPS::Signal;

ExciterStatic::ExciterStatic(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { }

void ExciterStatic::setParameters(std::shared_ptr<Base::ExciterParameters> parameters) {

	if (auto params = std::dynamic_pointer_cast<Signal::ExciterStaticParameters>(parameters)){
		mParameters = params;

		if (mParameters->Ta == mParameters->Tb) {
			SPDLOG_LOGGER_INFO(mSLog, 
				"\n if Ta=Tb  the auxilary state variable Xb can be ignored, as Xb=0",
				this->name());
			throw CPS::TypeException();
		}

		SPDLOG_LOGGER_INFO(mSLog, 
			"\nExciteStatic parameters:"
			"\nTa: {:e}"
			"\nTb: {:e}"
			"\nTe: {:e}"
			"\nKa: {:e}"
			"\nMaximum EMF: {:e}"
			"\nMinimum EMF: {:e}\n",
			mParameters->Ta, mParameters->Tb, 
			mParameters->Te, mParameters->Ka,
			mParameters->MaxEfd, mParameters->MinEfd);
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be ExciterStatic!" << std::endl;
		throw CPS::TypeException();
	}
}

void ExciterStatic::initialize(Real Vh_init, Real Ef_init) {
	
	mVh = Vh_init;
	mEfd = Ef_init;
	mEfd_next = Ef_init;

	SPDLOG_LOGGER_INFO(mSLog, 
		"Initially set excitation system initial values:"
		"\ninit Vh: {:e}"
		"\ninit Ef: {:e}",
		mVh, mEfd);

	if ((Ef_init>mParameters->MaxEfd) || (Ef_init<mParameters->MinEfd))
		SPDLOG_LOGGER_WARN(mSLog, 
			"\nWARNING: The initialisation is bad, Ef_init out of allowed band ({}<Ef_init<{})."
			"\nThe simulation will be continued ",
			mParameters->MinEfd, mParameters->MaxEfd);

	// initialize auxiliar parameters
	mCa = mParameters->Ta / mParameters->Tb;
	mCb = (mParameters->Tb - mParameters->Ta) / mParameters->Tb;

	///
	mVref = mVh + mEfd / mParameters->Ka;

	/// Input of the first lead lag block
	mVin = mVref - mVh;

	/// Initial value for auxilar state variable
	mXb = (mEfd / mParameters->Ka - mVin * mCa) / mCb ;
	mXb_next = mXb;
	
	SPDLOG_LOGGER_INFO(mSLog, 
		"\nExciter Initialization"
		"\nExciter type: ExciterStatic"
		"\nCalculated set poit and auxilary state variables:"
		"\nVref : {:e}"
		"\nXb : {:e}",
		"\nVin = Vref-Vh : {:e}",
		mVref, mXb, mVin);
	mSLog->flush();
}

Real ExciterStatic::step(Real mVd, Real mVq, Real dt, Real Vpss) {

	// Voltage magnitude calculation
	mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));

	/// Update state variables that were calculated in last step
	mEfd = mEfd_next;
	mXb = mXb_next;

	/// Compute Xb at time k+1 using euler forward
	mVin = mVref - mVh + Vpss;
	mXb_next = mXb + (mVin - mXb) * dt / mParameters->Tb;
	
	// Compute Edf at time k+1 using euler forward
	mVe = (mVin * mCa + mXb * mCb) * mParameters->Ka;
	mEfd_next = (mVe-mEfd) * dt / mParameters->Te + mEfd;

	/// Limiter for Efd
	if (mEfd_next > mParameters->MaxEfd)
		mEfd_next = mParameters->MaxEfd;
	else if (mEfd_next < mParameters->MinEfd)
		mEfd_next = mParameters->MinEfd;

	return mEfd_next;
}