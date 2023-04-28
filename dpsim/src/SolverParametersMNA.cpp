#include <chrono>
#include <iomanip>
#include <algorithm>
#include <typeindex>


#include <dpsim/SolverParametersMNA.h>
#include <dpsim/Utils.h>
#include <dpsim-models/Utils.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#ifdef WITH_CIM
  #include <dpsim-models/CIM/Reader.h>
#endif

#ifdef WITH_SUNDIALS
  #include <dpsim-models/Solver/ODEInterface.h>
  #include <dpsim/DAESolver.h>
  #include <dpsim/ODESolver.h>
#endif

using namespace CPS;
using namespace DPsim;

