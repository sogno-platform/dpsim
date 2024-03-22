#include <dpsim-models/Signal/DroopController.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

void DroopController::setParameters(std::shared_ptr<DroopControllerParameters> parameters)
{
	mParameters = parameters;
	SPDLOG_LOGGER_INFO(mSLog,
					   "\nDroopController Parameters:"
					   "\n\tM_p = {:e}"
					   "\n\tTau_p = {:e}"
					   "\n\tTau_l = {:e}",
					   mParameters->Mp,
					   mParameters->Tau_p, mParameters->Tau_l);
	mSLog->flush();
}

void DroopController::initialize(Real time_step, Real initial_omega)
{
	mTimeStep = time_step;
	**mOmega = initial_omega;
}

Real DroopController::step(const Real mActivePower)
{
	//
	**mOmega = **mOmega + mTimeStep / mParameters->Tau_p *
							  (-**mOmega + mParameters->OmegaNom - mParameters->Mp * (mParameters->Pref - mActivePower));

	return **mOmega;
}