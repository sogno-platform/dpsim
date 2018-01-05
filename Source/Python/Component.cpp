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

PyObject* Python::Component::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds)
{
	Component* self = (Component*) type->tp_alloc(type, 0);
	if (self) {
		Component::init(self);
	}

	return (PyObject*) self;
}

void Python::Component::init(Component* self)
{
	new (&self->comp) DPsim::Component::Base::Ptr(nullptr);
}

void Python::Component::dealloc(Python::Component* self)
{
	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using Ptr = DPsim::Component::Base::Ptr;

	self->comp.~Ptr();

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

	DPsim::Component::Attribute::Map& attrMap = self->comp->getAttrMap();

	auto search = attrMap.find(name);
	if (search == attrMap.end()) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", name);
		return nullptr;
	}

	DPsim::Component::Attribute attr = search->second;
	switch (attr.mType) {
	case DPsim::Component::Attribute::Real:
		return PyFloat_FromDouble(*((Real*) attr.mValue));
	case DPsim::Component::Attribute::Integer:
		return PyLong_FromLong(*((Int*) attr.mValue));
	case DPsim::Component::Attribute::String:
		return PyUnicode_FromString(((std::string*) attr.mValue)->c_str());
	case DPsim::Component::Attribute::Complex:
		Complex c = *((Complex*) attr.mValue);
		return PyComplex_FromDoubles(c.real(), c.imag());
	}

	PyErr_Format(PyExc_SystemError, "invalid type in internal attribute map");

	return nullptr;
}

int Python::Component::setattr(Python::Component* self, char* name, PyObject *v)
{
	Int i;
	Real r;

	if (!self->comp) {
		PyErr_SetString(PyExc_ValueError, "setattr on unitialized Component");
		return -1;
	}

	DPsim::Component::Attribute::Map& attrMap = self->comp->getAttrMap();
	auto search = attrMap.find(name);
	if (search == attrMap.end()) {
		PyErr_Format(PyExc_AttributeError, "Component has no attribute '%s'", name);
		return -1;
	}

	DPsim::Component::Attribute attr = search->second;
	switch (attr.mType) {
	case DPsim::Component::Attribute::Real:
		r = PyFloat_AsDouble(v);
		if (PyErr_Occurred())
			return -1;
		*((Real*) attr.mValue) = r;
		break;
	case DPsim::Component::Attribute::Integer:
		i = PyLong_AsLong(v);
		if (PyErr_Occurred())
			return -1;
		*((Int*) attr.mValue) = i;
		break;
	case DPsim::Component::Attribute::String:
		if (!PyUnicode_Check(v))
			return -1;
		*((std::string*) attr.mValue) = std::string(PyUnicode_AsUTF8(v));
		break;
	case DPsim::Component::Attribute::Complex:
		if (!PyComplex_Check(v))
			return -1;
		*((Complex*) attr.mValue) = Complex(PyComplex_RealAsDouble(v), PyComplex_ImagAsDouble(v));
		break;
	default:
		PyErr_Format(PyExc_SystemError, "invalid type in internal attribute map");
		return -1;
	}

	return 0;
}

bool Python::compsFromPython(PyObject* list, DPsim::Component::Base::List& comps)
{
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

