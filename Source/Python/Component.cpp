/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <stdexcept>

#include <dpsim/Python/Component.h>
#include <dpsim/Python/Node.h>

using namespace DPsim;

PyObject* Python::Component::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds)
{
	Component* self = (Component*) type->tp_alloc(type, 0);
	if (self) {
		init(self);
	}

	return (PyObject*) self;
}

void Python::Component::init(Component* self)
{
	new (&self->comp) CPS::IdentifiedObject::Ptr(nullptr);
}

void Python::Component::dealloc(Python::Component* self)
{
	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using Ptr = CPS::IdentifiedObject::Ptr;

	self->comp.~Ptr();

	Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* Python::Component::str(Python::Component* self)
{
	if (!self->comp)
		return PyUnicode_FromString("<unitialized Component>");

	return PyUnicode_FromString(self->comp->name().c_str());
}

PyObject* Python::Component::getattro(Python::Component* self, PyObject* name)
{
	PyObject *attr;

	// Check is there is already an attibute with this name
	attr = PyObject_GenericGetAttr((PyObject *) self, name);
	if (!attr) {
		if (!PyErr_ExceptionMatches(PyExc_AttributeError))
			return NULL;

		PyErr_Clear();
	}
	else
		return attr;

	if (!self->comp) {
		PyErr_SetString(PyExc_ValueError, "getattr on unitialized Component");
		return nullptr;
	}

	try {
		auto attr = self->comp->attribute(PyUnicode_AsUTF8(name));

		return attr->toPyObject();
	}
	catch (const CPS::InvalidAttributeException &) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", PyUnicode_AsUTF8(name));
		return nullptr;
	}
	catch (...) {
		PyErr_Format(PyExc_RuntimeError, "Unkown Error Occured", name);
		return nullptr;
	}
}

int Python::Component::setattro(Python::Component* self, PyObject *name, PyObject *v)
{
	if (!self->comp) {
		PyErr_SetString(PyExc_ValueError, "setattr on unitialized Component");
		return -1;
	}

	try {
		auto attr = self->comp->attribute(PyUnicode_AsUTF8(name));
		attr->fromPyObject(v);
	}
	catch (const CPS::InvalidAttributeException &) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", PyUnicode_AsUTF8(name));
		return -1;
	}
	catch (const CPS::TypeException &) {
		PyErr_Format(PyExc_TypeError, "Invalid type for attribute '%s'", PyUnicode_AsUTF8(name));
		return -1;
	}
	catch (const CPS::AccessException &) {
		PyErr_Format(PyExc_AttributeError, "Attribute '%s' is not modifiable", PyUnicode_AsUTF8(name));
		return -1;
	}
	catch (...) {
		PyErr_Format(PyExc_RuntimeError, "Unkown Error Occured", PyUnicode_AsUTF8(name));
		return -1;
	}

	return 0;
}

CPS::IdentifiedObject::List Python::compsFromPython(PyObject* list)
{
	CPS::IdentifiedObject::List comps;

	if (!PyList_Check(list))
		throw std::invalid_argument("argument must be a list");

	for (int i = 0; i < PyList_Size(list); i++) {
		PyObject* obj = PyList_GetItem(list, i);
		if (!PyObject_TypeCheck(obj, &Python::Component::type))
			throw std::invalid_argument( "list element is not a dpsim.Component" );

		Component* pyComp = (Component*) obj;
		comps.push_back(pyComp->comp);
	}

	return comps;
}

const char* Python::Component::docConnect = "";
PyObject* Python::Component::connect(Component* self, PyObject* args)
{
	PyObject *pyNodes;

	if (!PyArg_ParseTuple(args, "O", &pyNodes))
		return nullptr;

	try {
		using EMTComponent = CPS::SimPowerComp<CPS::Real>;
		using DPComponent = CPS::SimPowerComp<CPS::Complex>;

		if (auto emtComp = std::dynamic_pointer_cast<EMTComponent>(self->comp)) {
			auto nodes = Python::Node<CPS::Real>::fromPython(pyNodes);

			emtComp->connect(nodes);

		}
		else if (auto dpComp = std::dynamic_pointer_cast<DPComponent>(self->comp)) {
			auto nodes = Python::Node<CPS::Complex>::fromPython(pyNodes);

			dpComp->connect(nodes);
		}
		else {
			PyErr_SetString(PyExc_TypeError, "Failed to connect nodes");
			return nullptr;
		}

		Py_RETURN_NONE;
	} catch (...) {
		PyErr_SetString(PyExc_TypeError, "Failed to connect nodes");
		return nullptr;
	}
}

PyObject* Python::Component::dir(Component* self, PyObject* args) {
	auto compAttrs = self->comp->attributes();

	PyObject *pyAttrs, *pyAttrName;

	pyAttrs = PyList_New(0);

	for (auto it : compAttrs) {
		pyAttrName = PyUnicode_FromString(it.first.c_str());

		PyList_Append(pyAttrs, pyAttrName);
	}

	return pyAttrs;
}

PyMethodDef Python::Component::methods[] = {
	{"connect", (PyCFunction) Python::Component::connect, METH_VARARGS, Python::Component::docConnect},
	{"__dir__", (PyCFunction) Python::Component::dir, METH_NOARGS, nullptr},
	{0},
};

const char* Python::Component::doc =
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
PyTypeObject Python::Component::type = {
	PyVarObject_HEAD_INIT(nullptr, 0)
	"dpsim.Component",                         /* tp_name */
	sizeof(Python::Component),                 /* tp_basicsize */
	0,                                         /* tp_itemsize */
	(destructor)Python::Component::dealloc,    /* tp_dealloc */
	0,                                         /* tp_print */
	0,                                         /* tp_getattr */
	0,                                         /* tp_setattr */
	0,                                         /* tp_reserved */
	0,                                         /* tp_repr */
	0,                                         /* tp_as_number */
	0,                                         /* tp_as_sequence */
	0,                                         /* tp_as_mapping */
	0,                                         /* tp_hash  */
	0,                                         /* tp_call */
	(reprfunc)Python::Component::str,          /* tp_str */
	(getattrofunc)Python::Component::getattro, /* tp_getattro */
	(setattrofunc)Python::Component::setattro, /* tp_setattro */
	0,                                         /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,  /* tp_flags */
	Python::Component::doc,                    /* tp_doc */
	0,                                         /* tp_traverse */
	0,                                         /* tp_clear */
	0,                                         /* tp_richcompare */
	0,                                         /* tp_weaklistoffset */
	0,                                         /* tp_iter */
	0,                                         /* tp_iternext */
	Python::Component::methods,                /* tp_methods */
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
