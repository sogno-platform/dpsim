#ifndef PYINTERFACE_H
#define PYINTERFACE_H

#include <Python.h>

#include "ExternalInterface.h"

namespace DPsim {
	// Thin Python wrapper around ExternalInterface
	struct PyInterface {
		PyObject_HEAD

		ExternalInterface *intf;

		static void dealloc(PyInterface*);

		static PyObject* exportCurrent(PyObject* self, PyObject* args);
		static PyObject* exportVoltage(PyObject* self, PyObject* args);
		static PyObject* registerSource(PyObject* self, PyObject* args);
	};

	extern PyTypeObject PyInterfaceType;

#ifdef __linux__
	PyObject* pyShmemInterface(PyObject *self, PyObject *args, PyObject *kwds);
#endif
};

#endif
