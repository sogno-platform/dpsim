#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Logger.h>

namespace CPS
{
	namespace Signal
	{

		class DroopControllerParameters : public SharedFactory<DroopControllerParameters>
		{

		public:
			/// Time constant lag unit
			Real Tau_p;
			/// Time constant lead-lag unit
			Real Tau_l;
			///
			Real Mp;
			///
			Real Pref;
			///
			Real OmegaNom;
		};

		/// DOC:
		class DroopController : public SimSignalComp,
								public SharedFactory<DroopController>
		{

		private:
			///
			Real mTimeStep;
			/// Controller Parameters
			std::shared_ptr<DroopControllerParameters> mParameters;

			/// Controller variables
			/// Omega
			const Attribute<Real>::Ptr mOmega;

		public:
			///
			explicit DroopController(const String &name, CPS::Logger::Level logLevel) : SimSignalComp(name, name, logLevel),
																						mOmega(mAttributes->create<Real>("Omega", 0)) {}

			/// Sets Parameters of the vsi controller
			void setParameters(std::shared_ptr<DroopControllerParameters> parameters);

			/// Initialises the initial state of the vsi controller
			void initialize(Real time_step, Real initial_omega);

			/// Performs a step to update all state variables and the output
			Real step(const Real mActivePower);
		};

	}
}