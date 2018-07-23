/** Python Interface
 *
 * @file
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

#include "Config.h"

#ifdef WITH_SHMEM
  #include "cps/Interface.h"
#endif

#include "Python/Interface.h"
#include "Python/Component.h"

using namespace CPS;

PyObject* Python::Interface::newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	Python::Interface *self;

	self = (Python::Interface*) type->tp_alloc(type, 0);
	if (self) {
#ifdef WITH_SHMEM
		using SharedIntfPtr = std::shared_ptr<CPS::Interface>;

		new (&self->intf) SharedIntfPtr();
#endif
	}

	return (PyObject*) self;
}

void Python::Interface::dealloc(Python::Interface* self)
{
#ifdef WITH_SHMEM
		using SharedIntfPtr = std::shared_ptr<CPS::Interface>;

		self->intf.~SharedIntfPtr();
#endif

	Py_TYPE(self)->tp_free((PyObject*) self);
}

static const char* DocInterfaceRegisterControlledAttribute =
"import(comp, attr, real_idx, imag_idx)\n"
"Register a source with this interface, causing it to use values received from "
"this interface as its current or voltage value.\n"
"\n"
":param comp: The ``Component`` whose attribute we want to modify.\n"
":param attr:\n"
":param real_idx: Index of the real part of the current or voltage.\n"
":param imag_idx: Index of the imaginary part of the current or voltage.\n";
PyObject* Python::Interface::registerControlledAttribute(PyObject* self, PyObject* args)
{
#ifdef WITH_SHMEM
	PyObject *obj;
	int realIdx, imagIdx = -1;
	const char *attrName;
	double gain;

	Python::Interface *pyIntf = (Python::Interface*) self;

	if (!PyArg_ParseTuple(args, "Osfi|i", &obj, &attrName, &gain, &realIdx, &imagIdx))
		return nullptr;

	if (!PyObject_TypeCheck(obj, &Python::ComponentType)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component");
		return nullptr;
	}

	Python::Component *pyComp = (Python::Component*) obj;

	try {
		Attribute<Real>::Ptr realAttr = pyComp->comp->findAttribute<Real>(attrName);

		pyIntf->intf->addImport(realAttr, gain, realIdx);
	}
	catch (CPS::AttributeList::InvalidAttributeException) {
		try {
			Attribute<Complex>::Ptr compAttr = pyComp->comp->findAttribute<Complex>(attrName);

			if (imagIdx < 0) {
				PyErr_SetString(PyExc_TypeError, "Both a real and imaginary index must be specified");
				return nullptr;
			}

			pyIntf->intf->addImport(compAttr, gain, realIdx, imagIdx);
		}
		catch (CPS::AttributeList::InvalidAttributeException) {
			PyErr_SetString(PyExc_TypeError, "First argument must be a readable attribute");
			return nullptr;
		}
	}

	Py_INCREF(Py_None);

	return Py_None;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}

static const char* DocInterfaceRegisterExportedAttribute =
"export(comp, attr, real_idx, imag_idx)\n"
"Register a voltage between two nodes to be written to this interface after every timestep.\n"
"\n"
":param comp:\n"
":param attr:\n"
":param real_idx: Index where the real part of the voltage is written.\n"
":param imag_idx: Index where the imaginary part of the voltage is written.\n";
PyObject* Python::Interface::registerExportedAttribute(PyObject* self, PyObject* args)
{
#ifdef WITH_SHMEM
	PyObject* obj;
	int realIdx, imagIdx = -1;
	const char *attrName;
	double gain;

	Python::Interface* pyIntf = (Python::Interface*) self;

	if (!PyArg_ParseTuple(args, "Osfi|i", &obj, &attrName, &gain, &realIdx, &imagIdx)) {
		return nullptr;
	}

	if (!PyObject_TypeCheck(obj, &Python::ComponentType)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component");
		return nullptr;
	}

	Component *pyComp = (Component*) obj;

	try {
		Attribute<Real>::Ptr realAttr = pyComp->comp->findAttribute<Real>(attrName);

		pyIntf->intf->addExport(realAttr, gain, realIdx);
	}
	catch (CPS::AttributeList::InvalidAttributeException exp) {
		try {
			Attribute<Complex>::Ptr compAttr = pyComp->comp->findAttribute<Complex>(attrName);

			if (imagIdx < 0) {
				PyErr_SetString(PyExc_TypeError, "Both a real and imaginary index must be specified");
				return nullptr;
			}

			pyIntf->intf->addExport(compAttr, gain, realIdx, imagIdx);
		}
		catch (CPS::AttributeList::InvalidAttributeException exp) {
			PyErr_SetString(PyExc_TypeError, "First argument must be a writable attribute");
			return nullptr;
		}
	}

	Py_INCREF(Py_None);

	return Py_None;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}

const char* Python::DocOpenInterface =
"open_interface(wname, rname, queuelen=512, samplelen=64, polling=False)\n"
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
PyObject* Python::OpenInterface(PyObject *self, PyObject *args, PyObject *kwds)
{
#ifdef WITH_SHMEM
	static char *kwlist[] = {"wname", "rname", "queuelen", "samplelen", "polling", nullptr};
	CPS::Interface::Config conf;
	const char *wname, *rname;

	conf.queuelen = 512;
	conf.samplelen = 64;
	conf.polling = 0;
	if (!PyArg_ParseTupleAndKeywords(args, kwds, "ss|iib", kwlist,
		&wname, &rname, &conf.queuelen, &conf.samplelen, &conf.polling)) {
		return nullptr;
	}

	Python::Interface *pyIntf = PyObject_New(Python::Interface, &Python::InterfaceType);
	pyIntf->intf = CPS::Interface::make(wname, rname, &conf);
	return (PyObject*) pyIntf;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}

static PyMethodDef Interface_methods[] = {
	{"export_attribute", Python::Interface::registerExportedAttribute, METH_VARARGS, DocInterfaceRegisterExportedAttribute},
	{"import_attribute", Python::Interface::registerControlledAttribute, METH_VARARGS, DocInterfaceRegisterControlledAttribute},
	{0},
};

static const char* DocInterface =
"A connection to an external program, using which simulation data is exchanged.\n"
"\n"
"Currently, only an interface using POSIX shared memory is implemented (see `open_interface`).\n";
PyTypeObject Python::InterfaceType = {
	PyVarObject_HEAD_INIT(NULL, 0)
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
	DocInterface,                            /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Interface_methods,                       /* tp_methods */
	0,                                       /* tp_members */
	0,                                       /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	0,                                       /* tp_init */
	0,                                       /* tp_alloc */
	Python::Interface::newfunc               /* tp_new */
};
