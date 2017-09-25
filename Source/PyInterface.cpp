#include "PyComponent.h"
#include "PyInterface.h"

#include "ShmemInterface.h"

using namespace DPsim;

void PyInterface::dealloc(PyInterface* self) {
	if (self->intf)
		delete self->intf;
	Py_TYPE(self)->tp_free((PyObject*) self);
}

const char* pyDocInterfaceRegisterSource =
"register_source(source, real_idx, imag_idx)\n"
"Register a source with this interface, causing it to use values received from "
"this interface as its current or voltage value.\n"
"\n"
":param source: The ``ExternalCurrentSource`` or ``ExternalVoltageSource`` to be registered.\n"
":param real_idx: Index of the real part of the current or voltage.\n"
":param imag_idx: Index of the imaginary part of the current or voltage.\n";
PyObject* PyInterface::registerSource(PyObject* self, PyObject* args) {
	PyObject *obj;
	int realIdx, imagIdx;

	PyInterface* pyIntf = (PyInterface*) self;
	if (!PyArg_ParseTuple(args, "Oii", &obj, &realIdx, &imagIdx))
		return nullptr;

	if (!PyObject_TypeCheck(obj, &PyComponentType)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component");
		return nullptr;
	}
	PyComponent *pyComp = (PyComponent*) obj;
	if (ExternalCurrentSource *ecs = dynamic_cast<ExternalCurrentSource*>(pyComp->comp)) {
		pyIntf->intf->registerCurrentSource(ecs, realIdx, imagIdx);
	} else if (ExternalVoltageSource *evs = dynamic_cast<ExternalVoltageSource*>(pyComp->comp)) {
		pyIntf->intf->registerVoltageSource(evs, realIdx, imagIdx);
	} else {
		PyErr_SetString(PyExc_TypeError, "First argument must be an external source");
		return nullptr;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocInterfaceExportCurrent =
"export_current(comp, real_idx, imag_idx)\n"
"Register a current to be written to this interface after every timestep.\n"
"\n"
":param comp: The current flowing through this `Component` is exported. Note that only component types for which this curcent can easily be determined may be passed here.\n"
":param real_idx: Index where the real part of the current is written.\n"
":param imag_idx: Index where the imaginary part of the current is written.\n";
PyObject* PyInterface::exportCurrent(PyObject* self, PyObject* args) {
	int realIdx, imagIdx;
	PyObject* obj;
	PyInterface* pyIntf = (PyInterface*) self;

	if (!PyArg_ParseTuple(args, "Oii", &obj, &realIdx, &imagIdx))
		return nullptr;
	if (!PyObject_TypeCheck(obj, &PyComponentType)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a Component");
		return nullptr;
	}
	PyComponent *pyComp = (PyComponent*) obj;
	pyIntf->intf->registerExportedCurrent(pyComp->comp, realIdx, imagIdx);
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocInterfaceExportVoltage =
"export_voltage(from, to, real_idx, imag_idx)\n"
"Register a voltage between two nodes to be written to this interface after every timestep.\n"
"\n"
":param from: Number of the positive node of the voltage to be exported. The same "
"numbering systems as for the component constructors is used, e.g. 0 is the "
"ground, and indices for other nodes start at 1.\n"
":param to: Number of the negative node of the voltage to be exported.\n"
":param real_idx: Index where the real part of the voltage is written.\n"
":param imag_idx: Index where the imaginary part of the voltage is written.\n";
PyObject* PyInterface::exportVoltage(PyObject* self, PyObject* args) {
	int from, to, realIdx, imagIdx;
	PyInterface* pyIntf = (PyInterface*) self;

	if (!PyArg_ParseTuple(args, "iiii", &from, &to, &realIdx, &imagIdx))
		return nullptr;
	pyIntf->intf->registerExportedVoltage(from, to, realIdx, imagIdx);
	Py_INCREF(Py_None);
	return Py_None;
}

const char* DPsim::pyDocOpenShmemInterface =
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
PyObject* DPsim::pyOpenShmemInterface(PyObject *self, PyObject *args, PyObject *kwds) {
	static char *kwlist[] = {"wname", "rname", "queuelen", "samplelen", "polling", nullptr};
	struct shmem_conf conf;
	const char *wname, *rname;

	conf.queuelen = 512;
	conf.samplelen = 64;
	conf.polling = 0;
	if (!PyArg_ParseTupleAndKeywords(args, kwds, "ss|iib", kwlist,
		&wname, &rname, &conf.queuelen, &conf.samplelen, &conf.polling))
		return nullptr;

	PyInterface *pyIntf = PyObject_New(PyInterface, &PyInterfaceType);
	pyIntf->intf = new ShmemInterface(wname, rname, &conf);
	return (PyObject*) pyIntf;
}

static PyMethodDef PyInterface_methods[] = {
	{"export_current", PyInterface::exportCurrent, METH_VARARGS, pyDocInterfaceExportCurrent},
	{"export_voltage", PyInterface::exportVoltage, METH_VARARGS, pyDocInterfaceExportVoltage},
	{"register_source", PyInterface::registerSource, METH_VARARGS, pyDocInterfaceRegisterSource},
	{0},
};

const char* pyDocInterface =
"A connection to an external program, using which simulation data is exchanged.\n"
"\n"
"Currently, only an interface using POSIX shared memory is implemented (see `open_shmem_interface`).\n";
PyTypeObject DPsim::PyInterfaceType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"dpsim.Interface",                 /* tp_name */
	sizeof(PyInterface),               /* tp_basicsize */
	0,                                 /* tp_itemsize */
	(destructor)PyInterface::dealloc,  /* tp_dealloc */
	0,                                 /* tp_print */
	0,                                 /* tp_getattr */
	0,                                 /* tp_setattr */
	0,                                 /* tp_reserved */
	0,                                 /* tp_repr */
	0,                                 /* tp_as_number */
	0,                                 /* tp_as_sequence */
	0,                                 /* tp_as_mapping */
	0,                                 /* tp_hash  */
	0,                                 /* tp_call */
	0,                                 /* tp_str */
	0,                                 /* tp_getattro */
	0,                                 /* tp_setattro */
	0,                                 /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT |
		Py_TPFLAGS_BASETYPE,           /* tp_flags */
	pyDocInterface,                    /* tp_doc */
	0,                                 /* tp_traverse */
	0,                                 /* tp_clear */
	0,                                 /* tp_richcompare */
	0,                                 /* tp_weaklistoffset */
	0,                                 /* tp_iter */
	0,                                 /* tp_iternext */
	PyInterface_methods,               /* tp_methods */
};

