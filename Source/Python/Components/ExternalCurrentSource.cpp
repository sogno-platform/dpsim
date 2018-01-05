#include "Component.h"

using namespace DPsim;

const char *Python::DocExternalCurrentSource =
"ExternalCurrentSource(name, node1, node2, initial_current)\n"
"Construct a new external current source.\n"
"\n"
"An external current source is pretty much the same as a normal ideal current "
"source, but its current value can be controlled from external programs by "
"registering it with an `Interface`.\n"
"\n"
":param initial_current: The current of this source in the first timestep (as a complex value).\n"
":returns: A new `Component` representing this current source.\n";
PyObject* Python::ExternalCurrentSource(PyObject* self, PyObject* args) {
	const char *name;
	int src, dest;
	Py_complex initCurrent;

	if (!PyArg_ParseTuple(args, "siiD", &name, &src, &dest, &initCurrent))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ExternalCurrentSource>(name, src, dest, DPsim::Complex(initCurrent.real, initCurrent.imag));

	return (PyObject*) pyComp;
}
