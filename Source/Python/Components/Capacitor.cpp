#include "Component.h"

using namespace DPsim;

const char *Python::DocCapacitor =
"Capacitor(name, node1, node2, inductance)\n"
"Construct a new Capacitor.\n"
"\n"
"Attributes: ``capacitance``.\n"
"\n"
":param inductance: Inductance in Henry.\n"
":returns: A new `Component` representing this Capacitor.\n";

PyObject* Python::CapacitorDP(PyObject* self, PyObject* args) {
	const char *name;
	double capacitance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &capacitance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::Capacitor>(name, src, dest, capacitance);

	return (PyObject*) pyComp;
}
