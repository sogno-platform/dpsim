/** Python components
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include "Component.h"

#include "Components.h"

using namespace DPsim;

PyObject* Python::Component::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds) {
	Component* self = (Component*) type->tp_alloc(type, 0);
	if (self)
		Component::init(self);

	return (PyObject*) self;
}

void Python::Component::init(Component* self) {
	new (&self->comp) BaseComponent::Ptr(nullptr);
}

void Python::Component::dealloc(Python::Component* self) {
	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using Ptr = BaseComponent::Ptr;

	self->comp.~Ptr();

	Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* Python::Component::str(Python::Component* self) {
	if (!self->comp)
		return PyUnicode_FromString("<unitialized Component>");

	return PyUnicode_FromString(self->comp->getName().c_str());
}

PyObject* Python::Component::getattr(Python::Component* self, char* name) {
	if (!self->comp) {
		PyErr_SetString(PyExc_ValueError, "getattr on unitialized Component");
		return nullptr;
	}

	std::map<std::string, CompAttr>& attrMap = self->comp->getAttrMap();

	auto search = attrMap.find(name);
	if (search == attrMap.end()) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", name);
		return nullptr;
	}

	CompAttr attr = search->second;
	switch (attr.mType) {
	case AttrReal:
		return PyFloat_FromDouble(*((Real*) attr.value));
	case AttrInt:
		return PyLong_FromLong(*((Int*) attr.value));
	case AttrString:
		return PyUnicode_FromString(((std::string*) attr.value)->c_str());
	case AttrComplex:
		Complex c = *((Complex*) attr.value);
		return PyComplex_FromDoubles(c.real(), c.imag());
	}

	PyErr_Format(PyExc_SystemError, "invalid type in internal attribute map");

	return nullptr;
}

int Python::Component::setattr(Python::Component* self, char* name, PyObject *v) {
	Int i;
	Real r;

	if (!self->comp) {
		PyErr_SetString(PyExc_ValueError, "setattr on unitialized Component");
		return -1;
	}

	std::map<std::string, CompAttr>& attrMap = self->comp->getAttrMap();
	auto search = attrMap.find(name);
	if (search == attrMap.end()) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", name);
		return -1;
	}

	CompAttr attr = search->second;
	switch (attr.mType) {
	case AttrReal:
		r = PyFloat_AsDouble(v);
		if (PyErr_Occurred())
			return -1;
		*((Real*) attr.value) = r;
		break;
	case AttrInt:
		i = PyLong_AsLong(v);
		if (PyErr_Occurred())
			return -1;
		*((Int*) attr.value) = i;
		break;
	case AttrString:
		if (!PyUnicode_Check(v))
			return -1;
		*((std::string*) attr.value) = std::string(PyUnicode_AsUTF8(v));
		break;
	case AttrComplex:
		if (!PyComplex_Check(v))
			return -1;
		*((Complex*) attr.value) = Complex(PyComplex_RealAsDouble(v), PyComplex_ImagAsDouble(v));
		break;
	default:
		PyErr_Format(PyExc_SystemError, "invalid type in internal attribute map");
		return -1;
	}

	return 0;
}

bool Python::compsFromPython(PyObject* list, BaseComponent::List& comps) {
	if (!PyList_Check(list))
		return false;

	for (int i = 0; i < PyList_Size(list); i++) {
		PyObject* obj = PyList_GetItem(list, i);
		if (!PyObject_TypeCheck(obj, &Python::ComponentType)) {
			comps.clear();
			return false;
		}

		Component* pyComp = (Component*) obj;
		comps.push_back(pyComp->comp);
	}

	return true;
}

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

	Component *pyComp = PyObject_New(Component, &Python::ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ExternalCurrentSource>(name, src, dest, Complex(initCurrent.real, initCurrent.imag));

	return (PyObject*) pyComp;
}

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

	Component *pyComp = PyObject_New(Component, &Python::ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ExternalVoltageSource>(name, src, dest, Complex(initVoltage.real, initVoltage.imag), num);

	return (PyObject*) pyComp;
}

const char *Python::DocInductor =
"Inductor(name, node1, node2, inductance)\n"
"Construct a new inductor.\n"
"\n"
"Attributes: ``inductance``.\n"
"\n"
":param inductance: Inductance in Henry.\n"
":returns: A new `Component` representing this inductor.\n";
PyObject* Python::Inductor(PyObject* self, PyObject* args) {
	const char *name;
	double inductance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &inductance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &Python::ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::InductorDP>(name, src, dest, inductance);

	return (PyObject*) pyComp;
}

const char *Python::DocResistor =
"Resistor(name, node1, node2, resistance)\n"
"Construct a new resistor.\n"
"\n"
"Attributes: ``resistance``.\n"
"\n"
":param resistance: Resistance in Ohm.\n"
":returns: A new `Component` representing this resistor.\n";
PyObject* Python::Resistor(PyObject* self, PyObject* args) {
	const char *name;
	double resistance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &resistance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &Python::ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ResistorDP>(name, src, dest, resistance);

	return (PyObject*) pyComp;
}

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

	Component *pyComp = PyObject_New(Component, &Python::ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::VoltSourceRes>(name, src, dest, Complex(voltage.real, voltage.imag), resistance);

	return (PyObject*) pyComp;
}

static const char* DocComponent =
"A component of a network that is to be simulated.\n"
"\n"
"Instances of this class should either be created with the module-level "
"pseudo-constructors (like `Resistor`). The constructors all accept the same "
"first three arguments: ``name``, a simple string used for logging purposes, "
"and ``node1`` / ``node2``. "
#ifdef WITH_CIM
"Alternatively, the `load_cim` function can be used to construct components from "
"a Common Information Model (CIM) XML file. "
#endif
"These arguments are integers identifying the topological nodes that the component is connected "
"to. Normal indices start with 1 and must be sequential; the special index 0 "
"is used for the (always present) reference node with a fixed voltage of 0V.\n"
"\n"
"Most components have other parameters that are also accessible as attributes "
"after creation. These values must only be changed if the simulation is paused, "
"and `update_matrix` has to be called after changes are made.\n";
PyTypeObject Python::ComponentType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"dpsim.Component",                         /* tp_name */
	sizeof(Python::Component),                 /* tp_basicsize */
	0,                                         /* tp_itemsize */
	(destructor)Python::Component::dealloc,    /* tp_dealloc */
	0,                                         /* tp_print */
	(getattrfunc)Python::Component::getattr,   /* tp_getattr */
	(setattrfunc)Python::Component::setattr,   /* tp_setattr */
	0,                                         /* tp_reserved */
	0,                                         /* tp_repr */
	0,                                         /* tp_as_number */
	0,                                         /* tp_as_sequence */
	0,                                         /* tp_as_mapping */
	0,                                         /* tp_hash  */
	0,                                         /* tp_call */
	(reprfunc)Python::Component::str,          /* tp_str */
	0,                                         /* tp_getattro */
	0,                                         /* tp_setattro */
	0,                                         /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT |
		Py_TPFLAGS_BASETYPE,               /* tp_flags */
	DocComponent,                              /* tp_doc */
	0,                                         /* tp_traverse */
	0,                                         /* tp_clear */
	0,                                         /* tp_richcompare */
	0,                                         /* tp_weaklistoffset */
	0,                                         /* tp_iter */
	0,                                         /* tp_iternext */
	0,                                         /* tp_methods */
	0,                                         /* tp_members */
	0,                                         /* tp_getset */
	0,                                         /* tp_base */
	0,                                         /* tp_dict */
	0,                                         /* tp_descr_get */
	0,                                         /* tp_descr_set */
	0,                                         /* tp_dictoffset */
	0,                                         /* tp_init */
	0,                                         /* tp_alloc */
	Python::Component::newfunc,                /* tp_new */
};

