#ifndef PYCOMPONENT_H
#define PYCOMPONENT_H

#include <Python.h>

#include <vector>

#include "Components/BaseComponent.h"

namespace DPsim {

	struct PyComponent {
		PyObject_HEAD

		BaseComponent* comp;

		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static void dealloc(PyComponent*);
	};

	extern PyTypeObject PyComponentType;

	bool compsFromPython(PyObject* list, std::vector<BaseComponent*>& comps);
	PyObject* pyLoadCim(PyObject* self, PyObject* args);
};

#endif
