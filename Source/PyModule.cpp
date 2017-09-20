#include <Python.h>

#include "PyComponent.h"
#include "PyInterface.h"
#include "PyModule.h"
#include "PySimulation.h"

using namespace DPsim;

PyMethodDef DPsim::pyModuleMethods[] = {
	{"load_cim", pyLoadCim, METH_VARARGS, "Load a network from CIM file(s)."},
	{"ExternalCurrentSource", pyExternalCurrentSource, METH_VARARGS, "Construct a new external current source."},
	{"ExternalVoltageSource", pyExternalVoltageSource, METH_VARARGS, "Construct a new external voltage source."},
	{"ShmemInterface", (PyCFunction)pyShmemInterface, METH_VARARGS|METH_KEYWORDS, "Construct an Interface that communicates via POSIX shared memory."},
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
		PyInterfaceType.tp_new = PyType_GenericNew;
		if (PyType_Ready(&PyInterfaceType) < 0)
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

