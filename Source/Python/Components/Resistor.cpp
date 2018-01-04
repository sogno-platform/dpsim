#include "Component.h"

using namespace DPsim;

const char *Python::DocResistor =
"Resistor(name, node1, node2, resistance)\n"
"Construct a new resistor.\n"
"\n"
"Attributes: ``resistance``.\n"
"\n"
":param resistance: Resistance in Ohm.\n"
":returns: A new `Component` representing this resistor.\n";

PyObject* Python::ResistorDP(PyObject* self, PyObject* args) {
	const char *name;
	double resistance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &resistance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ResistorDP>(name, src, dest, resistance);

	return (PyObject*) pyComp;
}

PyObject* Python::ResistorEMT(PyObject* self, PyObject* args) {
	const char *name;
	double resistance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &resistance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ResistorEMT>(name, src, dest, resistance);

	return (PyObject*) pyComp;
}
