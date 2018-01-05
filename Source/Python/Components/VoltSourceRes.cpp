#include "Component.h"

using namespace DPsim;

const char *Python::DocVoltSourceRes =
"VoltSourceRes(name, node1, node2, voltage, resistance)\n"
"Construct a new voltage source with an internal resistance.\n"
"\n"
"Because this is actually internally represented as an equivalent current "
"source, it does **not** count towards the numbering of ideal voltage sources.\n"
"\n"
"Attributes: ``resistance``, ``voltage``.\n"
"\n"
":param voltage: Complex voltage in Volt.\n"
":param resistance: Internal resistance in Ohm.\n"
":returns: A new `Component` representing this voltage source.\n";
PyObject* Python::VoltSourceRes(PyObject* self, PyObject* args) {
	const char *name;
	double resistance;
	int src, dest;
	Py_complex voltage;

	if (!PyArg_ParseTuple(args, "siiDd", &name, &src, &dest, &voltage, &resistance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::VoltSourceRes>(name, src, dest, Complex(voltage.real, voltage.imag), resistance);

	return (PyObject*) pyComp;
}
