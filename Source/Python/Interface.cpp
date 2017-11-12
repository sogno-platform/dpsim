/** Python Interface
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

#include "Python/Component.h"
#include "Python/Interface.h"

#include "ShmemInterface.h"

using namespace DPsim;

void Python::Interface::dealloc(Python::Interface* self) {
	if (self->intf)
		delete self->intf;
	Py_TYPE(self)->tp_free((PyObject*) self);
}

static const char* DocInterfaceRegisterSource =
"register_source(source, real_idx, imag_idx)\n"
"Register a source with this interface, causing it to use values received from "
"this interface as its current or voltage value.\n"
"\n"
":param source: The ``ExternalCurrentSource`` or ``ExternalVoltageSource`` to be registered.\n"
":param real_idx: Index of the real part of the current or voltage.\n"
":param imag_idx: Index of the imaginary part of the current or voltage.\n";
PyObject* Python::Interface::registerSource(PyObject* self, PyObject* args) {
	PyObject *obj;
	int realIdx, imagIdx;

	Python::Interface* pyIntf = (Python::Interface*) self;
	if (!PyArg_ParseTuple(args, "Oii", &obj, &realIdx, &imagIdx))
		return nullptr;

	if (!PyObject_TypeCheck(obj, &Python::ComponentType)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component");
		return nullptr;
	}
	Python::Component *pyComp = (Python::Component*) obj;
	if (DPsim::ExternalCurrentSource *ecs = dynamic_cast<DPsim::ExternalCurrentSource*>(pyComp->comp.get())) {
		pyIntf->intf->registerCurrentSource(ecs, realIdx, imagIdx);
	} else if (DPsim::ExternalVoltageSource *evs = dynamic_cast<DPsim::ExternalVoltageSource*>(pyComp->comp.get())) {
		pyIntf->intf->registerVoltageSource(evs, realIdx, imagIdx);
	} else {
		PyErr_SetString(PyExc_TypeError, "First argument must be an external source");
		return nullptr;
	}

	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocInterfaceExportCurrent =
"export_current(comp, real_idx, imag_idx)\n"
"Register a current to be written to this interface after every timestep.\n"
"\n"
":param comp: The current flowing through this `Component` is exported. Note that only component types for which this curcent can easily be determined may be passed here.\n"
":param real_idx: Index where the real part of the current is written.\n"
":param imag_idx: Index where the imaginary part of the current is written.\n";
PyObject* Python::Interface::exportCurrent(PyObject* self, PyObject* args) {
	int realIdx, imagIdx;
	PyObject* obj;
	Python::Interface* pyIntf = (Python::Interface*) self;

	if (!PyArg_ParseTuple(args, "Oii", &obj, &realIdx, &imagIdx))
		return nullptr;
	if (!PyObject_TypeCheck(obj, &Python::ComponentType)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component");
		return nullptr;
	}

	Component *pyComp = (Component*) obj;
	pyIntf->intf->registerExportedCurrent(pyComp->comp.get(), realIdx, imagIdx);

	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocInterfaceExportVoltage =
"export_voltage(from, to, real_idx, imag_idx)\n"
"Register a voltage between two nodes to be written to this interface after every timestep.\n"
"\n"
":param from: Number of the positive node of the voltage to be exported. The same "
"numbering systems as for the component constructors is used, e.g. 0 is the "
"ground, and indices for other nodes start at 1.\n"
":param to: Number of the negative node of the voltage to be exported.\n"
":param real_idx: Index where the real part of the voltage is written.\n"
":param imag_idx: Index where the imaginary part of the voltage is written.\n";
PyObject* Python::Interface::exportVoltage(PyObject* self, PyObject* args) {
	int from, to, realIdx, imagIdx;
	Python::Interface* pyIntf = (Python::Interface*) self;

	if (!PyArg_ParseTuple(args, "iiii", &from, &to, &realIdx, &imagIdx))
		return nullptr;

	pyIntf->intf->registerExportedVoltage(from, to, realIdx, imagIdx);

	Py_INCREF(Py_None);

	return Py_None;
}

const char* Python::DocOpenShmemInterface =
"open_shmem_interface(wname, rname, queuelen=512, samplelen=64, polling=False)\n"
"Opens a set of shared memory regions to use as an interface for communication. The communication type / format of VILLASNode's shmem node is used; see its documentation for more information on the internals.\n"
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
PyObject* Python::OpenShmemInterface(PyObject *self, PyObject *args, PyObject *kwds) {

#ifdef __linux__
	static char *kwlist[] = {"wname", "rname", "queuelen", "samplelen", "polling", nullptr};
	struct shmem_conf conf;
	const char *wname, *rname;

	conf.queuelen = 512;
	conf.samplelen = 64;
	conf.polling = 0;
	if (!PyArg_ParseTupleAndKeywords(args, kwds, "ss|iib", kwlist,
		&wname, &rname, &conf.queuelen, &conf.samplelen, &conf.polling))
		return nullptr;

	Python::Interface *pyIntf = PyObject_New(Python::Interface, &Python::InterfaceType);
	pyIntf->intf = new ShmemInterface(wname, rname, &conf);
	return (PyObject*) pyIntf;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}

static PyMethodDef Interface_methods[] = {
	{"export_current", Python::Interface::exportCurrent, METH_VARARGS, DocInterfaceExportCurrent},
	{"export_voltage", Python::Interface::exportVoltage, METH_VARARGS, DocInterfaceExportVoltage},
	{"register_source", Python::Interface::registerSource, METH_VARARGS, DocInterfaceRegisterSource},
	{0},
};

static const char* DocInterface =
"A connection to an external program, using which simulation data is exchanged.\n"
"\n"
"Currently, only an interface using POSIX shared memory is implemented (see `open_shmem_interface`).\n";
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
	Py_TPFLAGS_DEFAULT |
		Py_TPFLAGS_BASETYPE,             /* tp_flags */
	DocInterface,                            /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Interface_methods,                       /* tp_methods */
};

