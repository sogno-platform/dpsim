#ifndef PYMODULE_H
#define PYMODULE_H

#include <Python.h>

namespace DPsim {
	extern PyMethodDef pyModuleMethods[];

	extern PyModuleDef dpsimModule;

};

// Has to be declared as extern C and without a namespace, because  the linker
// otherwise mangles the name so the Python interpreter can't find this function.
extern "C" {
	
#if(_WIN32 || _WIN64)
	extern __declspec(dllexport) PyObject* PyInit_dpsim(void);
#else
	extern PyObject* PyInit_dpsim(void);
#endif
};
#endif
