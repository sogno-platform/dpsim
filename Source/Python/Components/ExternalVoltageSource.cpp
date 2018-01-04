#include "Component.h"

using namespace DPsim;

const char *Python::DocExternalVoltageSource =
"ExternalVoltageSource(name, node1, node2, initial_voltage, num)\n"
"Construct a new external voltage source.\n"
"\n"
"An external voltage source is pretty much the same as a normal ideal voltage "
"source, but its voltage value can be controlled from external programs by "
"registering it with an `Interface`.\n"
"\n"
":param initial_current: The voltage of this source in the first timestep (as a complex value).\n"
":param num: The number of this voltage source. All ideal voltage sources must "
"be identified by sequential indices, starting with 1.\n"
":returns: A new `Component` representing this voltage source.\n";
PyObject* Python::ExternalVoltageSource(PyObject* self, PyObject* args) {
	const char *name;
	int src, dest, num;
	Py_complex initVoltage;

	if (!PyArg_ParseTuple(args, "siiDi", &name, &src, &dest, &initVoltage, &num))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ExternalVoltageSource>(name, src, dest, DPsim::Complex(initVoltage.real, initVoltage.imag), num);

	return (PyObject*) pyComp;
}
