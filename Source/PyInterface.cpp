#include "PyComponent.h"
#include "PyInterface.h"

#include "ShmemInterface.h"

using namespace DPsim;

static PyMethodDef PyInterface_methods[] = {
	{"export_current", PyInterface::exportCurrent, METH_VARARGS, "Registers a current to be exported on this interface."},
	{"export_voltage", PyInterface::exportVoltage, METH_VARARGS, "Registers a voltage to be exported on this interface."},
	{"register_source", PyInterface::registerSource, METH_VARARGS, "Registers an external source to use this interface."},
	{0},
};

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
	"An interface to an external source/sink of data.", /* tp_doc */
	0,                                 /* tp_traverse */
	0,                                 /* tp_clear */
	0,                                 /* tp_richcompare */
	0,                                 /* tp_weaklistoffset */
	0,                                 /* tp_iter */
	0,                                 /* tp_iternext */
	PyInterface_methods,               /* tp_methods */
};

void PyInterface::dealloc(PyInterface* self) {
	if (self->intf)
		delete self->intf;
	Py_TYPE(self)->tp_free((PyObject*) self);
}

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

PyObject* PyInterface::exportVoltage(PyObject* self, PyObject* args) {
	int from, to, realIdx, imagIdx;
	PyInterface* pyIntf = (PyInterface*) self;

	if (!PyArg_ParseTuple(args, "iiii", &from, &to, &realIdx, &imagIdx))
		return nullptr;
	pyIntf->intf->registerExportedVoltage(from, to, realIdx, imagIdx);
	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* DPsim::pyShmemInterface(PyObject *self, PyObject *args, PyObject *kwds) {
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
