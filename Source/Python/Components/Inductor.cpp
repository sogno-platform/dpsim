#include "Component.h"

using namespace DPsim;

const char *Python::DocInductor =
"Inductor(name, node1, node2, inductance)\n"
"Construct a new inductor.\n"
"\n"
"Attributes: ``inductance``.\n"
"\n"
":param inductance: Inductance in Henry.\n"
":returns: A new `Component` representing this inductor.\n";

PyObject* Python::InductorDP(PyObject* self, PyObject* args) {
	const char *name;
	double inductance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &inductance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::InductorDP>(name, src, dest, inductance);

	return (PyObject*) pyComp;
}

PyObject* Python::InductorEMT(PyObject* self, PyObject* args) {
	const char *name;
	double inductance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &inductance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::InductorEMT>(name, src, dest, inductance);

	return (PyObject*) pyComp;
}
