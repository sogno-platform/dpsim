#include <Python.h>

#include "PyComponent.h"
#include "PyInterface.h"
#include "PyModule.h"
#include "PySimulation.h"

using namespace DPsim;

PyMethodDef DPsim::pyModuleMethods[] = {
	{"load_cim", pyLoadCim, METH_VARARGS, pyDocLoadCim},
	{"open_shmem_interface", (PyCFunction)pyOpenShmemInterface, METH_VARARGS|METH_KEYWORDS, pyDocOpenShmemInterface},
	{"ExternalCurrentSource", pyExternalCurrentSource, METH_VARARGS, pyDocExternalCurrentSource},
	{"ExternalVoltageSource", pyExternalVoltageSource, METH_VARARGS, pyDocExternalVoltageSource},
	{"Inductor", pyInductor, METH_VARARGS, pyDocInductor},
	{"LinearResistor", pyLinearResistor, METH_VARARGS, pyDocLinearResistor},
	{"VoltSourceRes", pyVoltSourceRes, METH_VARARGS, pyDocVoltSourceRes},
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
		Py_INCREF(&PyInterfaceType);
		PyModule_AddObject(m, "Interface", (PyObject*) &PyInterfaceType);
		return m;
	}
};

