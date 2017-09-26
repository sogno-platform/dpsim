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

	// "Constructors" for the component types
	extern const char* pyDocExternalCurrentSource;
	PyObject* pyExternalCurrentSource(PyObject* self, PyObject *args);
	extern const char* pyDocExternalVoltageSource;
	PyObject* pyExternalVoltageSource(PyObject* self, PyObject *args);
	extern const char* pyDocInductor;
	PyObject* pyInductor(PyObject* self, PyObject *args);
	extern const char* pyDocLinearResistor;
	PyObject* pyLinearResistor(PyObject* self, PyObject *args);
	extern const char* pyDocVoltSourceRes;
	PyObject* pyVoltSourceRes(PyObject* self, PyObject *args);

	extern const char* pyDocLoadCim;
	PyObject* pyLoadCim(PyObject* self, PyObject* args);
};

#endif
