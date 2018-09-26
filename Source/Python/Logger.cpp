/** Python Loggers
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/Config.h>

#include <dpsim/DataLogger.h>
#include <dpsim/Python/Logger.h>
#include <dpsim/Python/Component.h>
#include <cps/AttributeList.h>

#include "structmember.h"

using namespace DPsim;

PyObject* Python::Logger::newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	Python::Logger *self;

	self = (Python::Logger*) type->tp_alloc(type, 0);
	if (self) {
		using SharedLoggerPtr = std::shared_ptr<DPsim::DataLogger>;

		new (&self->logger) SharedLoggerPtr();
	}

	return (PyObject*) self;
}

void Python::Logger::dealloc(Python::Logger* self)
{
	using SharedLoggerPtr = std::shared_ptr<DPsim::DataLogger>;

	self->logger.~SharedLoggerPtr();

	Py_TYPE(self)->tp_free((PyObject*) self);
}

const char* Python::Logger::docLogAttribute =
"log_attribute(comp, attr)\n"
"Register a source with this Logger, causing it to use values received from "
"this Logger as its current or voltage value.\n"
"\n"
":param comp: The ``Component`` whose attribute we want to modify.\n"
":param attr:\n"
":param real_idx: Index of the real part of the current or voltage.\n"
":param imag_idx: Index of the imaginary part of the current or voltage.\n";
PyObject* Python::Logger::logAttribute(Logger* self, PyObject* args, PyObject *kwargs)
{
	PyObject *pyObj;
	const char *attrName;

	const char *kwlist[] = {"component", "attribute", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "Os", (char **) kwlist, &pyObj, &attrName))
		return nullptr;

	CPS::AttributeList::Ptr attrList;
	if (PyObject_TypeCheck(pyObj, &Python::Component::type)) {
		auto *pyComp = (Component*) pyObj;

		attrList = std::dynamic_pointer_cast<CPS::AttributeList>(pyComp->comp);
	}
	else if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Real>::type)) {
		auto *pyNode = (Node<CPS::Real> *) pyObj;

		attrList = std::dynamic_pointer_cast<CPS::AttributeList>(pyNode->node);
	}
	else if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Complex>::type)) {
		auto *pyNode = (Node<CPS::Complex> *) pyObj;

		attrList = std::dynamic_pointer_cast<CPS::AttributeList>(pyNode->node);
	}
	else {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component or a Node");
		return nullptr;
	}

	try {
		auto a = attrList->attribute<CPS::Real>(attrName);

		self->logger->addAttribute(attrName, a);
	}
	catch (const CPS::InvalidAttributeException &exp) {
		PyErr_SetString(PyExc_TypeError, "Second argument must be a readable attribute");
		return nullptr;
	}

	Py_RETURN_NONE;
}

int Python::Logger::init(Python::Logger *self, PyObject *args, PyObject *kwds)
{
	static const char *kwlist[] = {"filename", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", (char **) kwlist, &self->filename)) {
		return -1;
	}

	self->logger = DPsim::DataLogger::make(self->filename);

	return 0;
}

PyMemberDef Python::Logger::members[] = {
	{(char *) "filename", T_STRING, offsetof(Python::Logger, filename), READONLY, nullptr},
	{nullptr}
};

PyMethodDef Python::Logger::methods[] = {
	{"log_attribute", (PyCFunction) Python::Logger::logAttribute, METH_VARARGS | METH_KEYWORDS, Python::Logger::docLogAttribute},
	{nullptr},
};

const char* Python::Logger::doc =
"__init__(filename)\n";
PyTypeObject Python::Logger::type = {
	PyVarObject_HEAD_INIT(nullptr, 0)
	"dpsim.Logger",                          /* tp_name */
	sizeof(Python::Logger),                  /* tp_basicsize */
	0,                                       /* tp_itemsize */
	(destructor)Python::Logger::dealloc,     /* tp_dealloc */
	0,                                       /* tp_print */
	0,                                       /* tp_getattr */
	0,                                       /* tp_setattr */
	0,                                       /* tp_reserved */
	0,                                       /* tp_repr */
	0,                                       /* tp_as_number */
	0,                                       /* tp_as_sequence */
	0,                                       /* tp_as_mapping */
	0,                                       /* tp_hash  */
	0,                                       /* tp_call */
	0,                                       /* tp_str */
	0,                                       /* tp_getattro */
	0,                                       /* tp_setattro */
	0,                                       /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,/* tp_flags */
	Python::Logger::doc,                     /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Python::Logger::methods,                 /* tp_methods */
	Python::Logger::members,                 /* tp_members */
	0,                                       /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc)Python::Logger::init,          /* tp_init */
	0,                                       /* tp_alloc */
	Python::Logger::newfunc                  /* tp_new */
};
