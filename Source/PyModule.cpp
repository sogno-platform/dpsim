#include <Python.h>

#include "PyComponent.h"
#include "PyModule.h"
#include "PySimulation.h"

using namespace DPsim;

PyMethodDef DPsim::pyModuleMethods[] = {
	{"load_cim", pyLoadCim, METH_VARARGS, "Load a network from CIM file(s)."},
	{0}
};

PyModuleDef DPsim::dpsimModule = {
	PyModuleDef_HEAD_INIT, "dpsim", NULL, -1, pyModuleMethods,
	NULL, NULL, NULL, NULL
};

extern "C" {
	PyObject* PyInit_dpsim(void) {
		PyObject* m;

		if (PyType_Ready(&PyComponentType) < 0)
			return nullptr;
		if (PyType_Ready(&PySimulationType) < 0)
			return nullptr;

		m = PyModule_Create(&dpsimModule);
		if (!m)
			return nullptr;

		Py_INCREF(&PySimulationType);
		PyModule_AddObject(m, "Simulation", (PyObject*) &PySimulationType);
		Py_INCREF(&PyComponentType);
		PyModule_AddObject(m, "Component", (PyObject*) &PyComponentType);
		return m;
	}
};

