#include <dpsim/MNASolverDynInterface.h>
#include "example.h"

#include <cstring>
#include <iostream>


MnaSolverPluginExample::MnaSolverPluginExample() : log(nullptr) {};
MnaSolverPluginExample::~MnaSolverPluginExample() {};

void MnaSolverPluginExample::set_logger(void (*logger)(const char *))
{
    log = logger;
    log("set_logger");
}

int MnaSolverPluginExample::initialize(
			int row_number,
			int nnz,
			double *csrValues,
			int *csrRowPtr,
			int *csrColInd)
{
    log("initialize");
    return 0;
}

int MnaSolverPluginExample::solve(
			double *rhs_values,
			double *lhs_values)
{
    log("solve");
    return 0;
}

static MnaSolverPluginExample* pluginSingleton = nullptr;

DPsimPlugin::MnaSolverDynInterface* getMNASolverPlugin(const char *name)
{
    if (pluginSingleton != nullptr) {
        return pluginSingleton;
    }

    if (name == nullptr || strcmp(name, PLUGIN_NAME) != 0) {
        std::cout << "error: name mismatch";
        return nullptr;
    }
	pluginSingleton = new MnaSolverPluginExample();
    return pluginSingleton;
}


void MnaSolverPluginExample::cleanup()
{
    log("cleanup");
    delete pluginSingleton;
    pluginSingleton = nullptr;
}
