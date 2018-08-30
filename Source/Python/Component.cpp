/** Python components
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <stdexcept>

#include "Python/Component.h"
#include "Python/Node.h"
#include "cps/Components.h"

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
	new (&self->comp) CPS::ComponentBase::Ptr(nullptr);
	new (&self->refs) std::vector<PyObject *>();
}

void Python::Component::dealloc(Python::Component* self)
{
	for (auto it : self->refs) {
		Py_DECREF(it);
	}

	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using Ptr = CPS::ComponentBase::Ptr;
	using PyObjectsList = std::vector<PyObject *>;

	self->comp.~Ptr();
	self->refs.~PyObjectsList();

	Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* Python::Component::str(Python::Component* self)
{
	if (!self->comp)
		return PyUnicode_FromString("<unitialized Component>");

	return PyUnicode_FromString(self->comp->getName().c_str());
}

PyObject* Python::Component::getattr(Python::Component* self, char* name)
{
	if (!self->comp) {
		PyErr_SetString(PyExc_ValueError, "getattr on unitialized Component");
		return nullptr;
	}

	try {
		auto attr = self->comp->findAttribute(name);

		return attr->toPyObject();
	}
	catch (const CPS::InvalidAttributeException &) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", name);
		return NULL;
	}
	catch (...) {
		PyErr_Format(PyExc_RuntimeError, "Unkown Error Occured", name);
		return NULL;
	}
}

int Python::Component::setattr(Python::Component* self, char* name, PyObject *v)
{
	if (!self->comp) {
		PyErr_SetString(PyExc_ValueError, "setattr on unitialized Component");
		return -1;
	}

	try {
		auto attr = self->comp->findAttribute(name);
		attr->fromPyObject(v);
	}
	catch (const CPS::InvalidAttributeException &) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", name);
		return -1;
	}
	catch (const CPS::TypeException &) {
		PyErr_Format(PyExc_TypeError, "Invalid type for attribute '%s'", name);
		return -1;
	}
	catch (const CPS::AccessException &) {
		PyErr_Format(PyExc_AttributeError, "Attribute '%s' is not modifiable", name);
		return -1;
	}
	catch (...) {
		PyErr_Format(PyExc_RuntimeError, "Unkown Error Occured", name);
		return -1;
	}

	return 0;
}

CPS::ComponentBase::List Python::compsFromPython(PyObject* list)
{
	CPS::ComponentBase::List comps;

	if (!PyList_Check(list))
		throw std::invalid_argument("argument must be a list");

	for (int i = 0; i < PyList_Size(list); i++) {
		PyObject* obj = PyList_GetItem(list, i);
		if (!PyObject_TypeCheck(obj, &Python::ComponentType))
			throw std::invalid_argument( "list element is not a dpsim.Component" );

		Component* pyComp = (Component*) obj;
		comps.push_back(pyComp->comp);
	}

	return comps;
}

static const char* DocComponentConnect = "";
PyObject* Python::Component::connect(PyObject* self, PyObject* args)
{
	Python::Component *pyComp = reinterpret_cast<Python::Component *>(self);
	PyObject *pyNodes;

	if (!PyArg_ParseTuple(args, "O", &pyNodes))
		return nullptr;

	try {
		using EMTComponent = CPS::PowerComponent<CPS::Real>;
		using DPComponent = CPS::PowerComponent<CPS::Complex>;

		if (auto emtComp = std::dynamic_pointer_cast<EMTComponent>(pyComp->comp)) {
			auto nodes = Python::Node<CPS::Real>::fromPython(pyNodes);

			emtComp->setNodes(nodes);

		}
		else if (auto dpComp = std::dynamic_pointer_cast<DPComponent>(pyComp->comp)) {
			auto nodes = Python::Node<CPS::Complex>::fromPython(pyNodes);

			dpComp->setNodes(nodes);
		}
		else {
			PyErr_SetString(PyExc_TypeError, "Failed to connect nodes");
			return nullptr;
		}

		Py_INCREF(Py_None);
		return Py_None;
	} catch (...) {
		PyErr_SetString(PyExc_TypeError, "Failed to connect nodes");
		return nullptr;
	}
}

static PyMethodDef Component_methods[] = {
	{"connect", Python::Component::connect, METH_VARARGS, DocComponentConnect},
	{0},
};


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
	Component_methods,                         /* tp_methods */
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

