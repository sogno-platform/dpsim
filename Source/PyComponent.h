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

		static PyObject* str(PyComponent* self);

		static PyObject* getattr(PyComponent* self, char* name);
		static int setattr(PyComponent *self, char* name, PyObject *v);
	};

	extern PyTypeObject PyComponentType;

	bool compsFromPython(PyObject* list, std::vector<BaseComponent*>& comps);

	PyObject* pyExternalCurrentSource(PyObject* self, PyObject *args);
	PyObject* pyExternalVoltageSource(PyObject* self, PyObject *args);
	PyObject* pyLoadCim(PyObject* self, PyObject* args);
};

#endif
