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
	};

	extern PyTypeObject PyInterfaceType;

#ifdef __linux__
	PyObject* pyShmemInterface(PyObject *self, PyObject *args);
#endif
};

#endif
