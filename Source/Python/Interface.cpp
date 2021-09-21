/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/Config.h>

#include <dpsim/Python/Interface.h>
#include <dpsim/Python/Component.h>

#include "structmember.h"

using namespace DPsim;

void Python::Interface::addExportDesc(Python::Interface* self, int idx, const CPS::String &type, const CPS::String &name, const CPS::String &suffix)
{
	while (PyList_Size(self->pyExports) < idx + 1)
		PyList_Append(self->pyExports, PyDict_New());

	PyObject *pyExport = PyDict_New();
	PyDict_SetItemString(pyExport, "name", PyUnicode_FromFormat("%s%s", name.c_str(), suffix.c_str()));
	PyDict_SetItemString(pyExport, "type", PyUnicode_FromString(type.c_str()));

	PyList_SetItem(self->pyExports, idx, pyExport);
}

PyObject* Python::Interface::newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	Python::Interface *self;

	self = (Python::Interface*) type->tp_alloc(type, 0);
	if (self) {
		using SharedIntfPtr = std::shared_ptr<DPsim::Interface>;

		new (&self->intf) SharedIntfPtr();

		self->pyExports = PyList_New(0);
	}

	return (PyObject*) self;
}

void Python::Interface::dealloc(Python::Interface* self)
{
		using SharedIntfPtr = std::shared_ptr<DPsim::Interface>;

		self->intf.~SharedIntfPtr();

		Py_DECREF(self->pyExports);

	Py_TYPE(self)->tp_free((PyObject*) self);
}

const char* Python::Interface::docAddImport =
"import(comp, attr, real_idx, imag_idx)\n"
"Register a source with this interface, causing it to use values received from "
"this interface as its current or voltage value.\n"
"\n"
":param comp: The ``Component`` whose attribute we want to modify.\n"
":param attr:\n"
":param real_idx: Index of the real part of the current or voltage.\n"
":param imag_idx: Index of the imaginary part of the current or voltage.\n";
PyObject* Python::Interface::addImport(Interface* self, PyObject* args, PyObject *kwargs)
{
	PyObject *pyObj;
	int idx;
	const char *attrName;
	double gain;
	int mode = 0;

	const char *kwlist[] = {"component", "attribute", "idx", "gain", "mode", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "Osi|ds", (char **) kwlist, &pyObj, &attrName, &idx, &gain, &mode))
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
		switch (mode) {
			case 0: { // Real Attribute
				auto a = self->intf->importReal(idx);
				attrList->setAttributeRef(attrName, a);
				break;
			}
			case 1: { // Complex Attribute: real & imag
				auto a = self->intf->importComplex(idx);
				attrList->setAttributeRef(attrName, a);
				break;
			}
			case 2: { // Complex Attribute: mag & phase
				auto a = self->intf->importComplexMagPhase(idx);

				attrList->setAttributeRef(attrName, a);
				break;
			}
			default:
				PyErr_SetString(PyExc_TypeError, "Invalid mode");
				return nullptr;
		}
	}
	catch (const CPS::InvalidAttributeException &exp) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a writable attribute");
		return nullptr;
	}

	Py_RETURN_NONE;
}

const char* Python::Interface::docAddExport =
"export(comp, attr, real_idx, imag_idx)\n"
"Register a voltage between two nodes to be written to this interface after every timestep.\n"
"\n"
":param comp:\n"
":param attr:\n"
":param real_idx: Index where the real part of the voltage is written.\n"
":param imag_idx: Index where the imaginary part of the voltage is written.\n";
PyObject* Python::Interface::addExport(Interface* self, PyObject* args, PyObject* kwargs)
{
	PyObject* pyObj;
	int idx;
	const char *attrName;
	double gain = 1;
	int mode = 0;
	int row = 0, col = 0;

	const char *kwlist[] = {"component", "attribute", "idx", "mode", "gain", "row", "col", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "Osi|idii", (char **) kwlist, &pyObj, &attrName, &idx, &mode, &gain, &row, &col))
		return nullptr;

	CPS::IdentifiedObject::Ptr obj;
	if (PyObject_TypeCheck(pyObj, &Python::Component::type)) {
		auto *pyComp = (Component*) pyObj;

		obj = std::dynamic_pointer_cast<CPS::IdentifiedObject>(pyComp->comp);
	}
	else if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Real>::type)) {
		auto *pyNode = (Node<CPS::Real> *) pyObj;

		obj = std::dynamic_pointer_cast<CPS::IdentifiedObject>(pyNode->node);
	}
	else if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Complex>::type)) {
		auto *pyNode = (Node<CPS::Complex> *) pyObj;

		obj = std::dynamic_pointer_cast<CPS::IdentifiedObject>(pyNode->node);
	}
	else {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component or a Node");
		return nullptr;
	}

	CPS::String name = obj->name() + "." + attrName;

	try {
		switch (mode) {
			case 0: { // Real Attribute
				auto realAttr = obj->attribute<CPS::Real>(attrName);

				self->intf->exportReal(realAttr, idx);
				addExportDesc(self, idx, "float", name);
				break;
			}

			case 1: { // Complex Attribute
				auto compAttr = obj->attribute<CPS::Complex>(attrName);

				self->intf->exportComplex(compAttr, idx);
				addExportDesc(self, idx, "complex", name);
				break;
			}

			case 2: { // Complex Attribute: mag & phase
				auto a = obj->attributeComplex(attrName);

				self->intf->exportReal(a->mag(),   idx);
				self->intf->exportReal(a->phase(), idx+1);
				addExportDesc(self, idx,   "float", name + ".mag");
				addExportDesc(self, idx+1, "float", name + ".phase");
				break;
			}

			case 3: { // Complex Attribute: real part
				auto a = obj->attributeComplex(attrName);

				self->intf->exportReal(a->real(), idx);
				addExportDesc(self, idx, "float", name, ".real");
				break;
			}

			case 4: { // Complex Attribute: imag part
				auto a = obj->attributeComplex(attrName);

				self->intf->exportReal(a->imag(), idx);
				addExportDesc(self, idx, "float", name + ".imag");
				break;
			}

			case 5: { // Complex Attribute: phase
				auto a = obj->attributeComplex(attrName);

				self->intf->exportReal(a->phase(), idx);
				addExportDesc(self, idx, "float", name + ".phase");
				break;
			}

			case 6: { // Complex Attribute: magnitude
				auto a = obj->attributeComplex(attrName);

				self->intf->exportReal(a->mag(), idx);
				addExportDesc(self, idx, "float", name + ".mag");
				break;
			}

			case 7: {
				auto a = obj->attributeMatrix<CPS::Real>(attrName);
				auto c = a->coeff(row, col);

				self->intf->exportReal(c, idx);
				addExportDesc(self, idx, "float", name + "(" + std::to_string(row) + "," + std::to_string(col) + ")");
				break;
			}

			case 8: {
				auto a = obj->attributeMatrix<CPS::Complex>(attrName);
				auto c = a->coeff(row, col);

				self->intf->exportComplex(c, idx);
				addExportDesc(self, idx, "complex", name + "(" + std::to_string(row) + "," + std::to_string(col) + ")");
				break;
			}

			case 9: {
				auto a = obj->attributeMatrix<CPS::Complex>(attrName);
				auto c = a->coeff(row, col);
				auto z = std::static_pointer_cast<CPS::ComplexAttribute>(c);

				self->intf->exportReal(z->mag(),   idx);
				self->intf->exportReal(z->phase(), idx+1);
				addExportDesc(self, idx,   "float", name + "(" + std::to_string(row) + "," + std::to_string(col) + ").mag");
				addExportDesc(self, idx+1, "float", name + "(" + std::to_string(row) + "," + std::to_string(col) + ").phase");
				break;
			}

			default:
				PyErr_SetString(PyExc_TypeError, "Invalid mode");
				return nullptr;
		}
	}
	catch (const CPS::InvalidAttributeException &exp) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a readable attribute");
		return nullptr;
	}

	Py_RETURN_NONE;
}

int Python::Interface::init(Python::Interface *self, PyObject *args, PyObject *kwds)
{
	static const char *kwlist[] = {"wname", "rname", "queuelen", "samplelen", "polling", nullptr};

	/* Default values */
	self->conf.queuelen = 512;
	self->conf.samplelen = 64;
	self->conf.polling = 0;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "ss|iib", (char **) kwlist,
		&self->wname, &self->rname, &self->conf.queuelen, &self->conf.samplelen, &self->conf.polling)) {
		return -1;
	}

	// instantiate specific interface here
	//self->intf = std::make_shared<DPsim::Interface>(self->wname, self->rname, &self->conf);

	return 0;
}

PyMemberDef Python::Interface::members[] = {
	{(char *) "wname",     T_STRING, offsetof(Python::Interface, wname),          READONLY, nullptr},
	{(char *) "rname",     T_STRING, offsetof(Python::Interface, rname),          READONLY, nullptr},
	{(char *) "queuelen",  T_INT,    offsetof(Python::Interface, conf.queuelen),  READONLY, nullptr},
	{(char *) "samplelen", T_INT,    offsetof(Python::Interface, conf.samplelen), READONLY, nullptr},
	{(char *) "polling",   T_INT,    offsetof(Python::Interface, conf.polling),   READONLY, nullptr},
	{(char *) "exports",   T_OBJECT, offsetof(Python::Interface, pyExports),      READONLY, nullptr},
	{nullptr}
};

PyMethodDef Python::Interface::methods[] = {
	{"export_attribute", (PyCFunction) Python::Interface::addExport, METH_VARARGS | METH_KEYWORDS, Python::Interface::docAddExport},
	{"import_attribute", (PyCFunction) Python::Interface::addImport, METH_VARARGS, Python::Interface::docAddImport},
	{nullptr},
};

const char* Python::Interface::doc =
"__init__(wname, rname, queuelen=512, samplelen=64, polling=False)\n"
"Opens a set of shared memory regions to use as an interface for communication. The communication type / format of VILLASNode's shmem node-type is used; see its documentation for more information on the internals.\n"
"\n"
"For this interface type, the indices passed to the methods specify the indices "
"in the array of values that is passed in each timestep. The sending program "
"must be configured to use the same indices as well.\n"
"\n"
":param wname: Name of the POSIX shared memory object where values are written. "
"Must start with a ``/``.\n"
":param rname: Name of the POSIX shared memory object where values are read from. "
"Must start with a ``/``.\n"
":param queuelen: Length of the outgoing queue as an integer number of samples.\n"
":param samplelen: Number of values that are passed as a \"sample\" each timestep. "
"Must be greater or equal to the maximum index passed to the export methods.\n"
":param polling: If True, no POSIX CV will be used to signal writes to the "
"interface, meaning that polling will have to be used. This may increase "
"performance at the cost of wasted CPU time.\n"
":returns: A new `Interface` object.\n";
PyTypeObject Python::Interface::type = {
	PyVarObject_HEAD_INIT(nullptr, 0)
	"dpsim.Interface",                       /* tp_name */
	sizeof(Python::Interface),               /* tp_basicsize */
	0,                                       /* tp_itemsize */
	(destructor)Python::Interface::dealloc,  /* tp_dealloc */
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
	Python::Interface::doc,                  /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Python::Interface::methods,              /* tp_methods */
	Python::Interface::members,              /* tp_members */
	0,                                       /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc)Python::Interface::init,       /* tp_init */
	0,                                       /* tp_alloc */
	Python::Interface::newfunc               /* tp_new */
};
