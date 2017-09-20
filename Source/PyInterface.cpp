#include "PyInterface.h"

#include "ShmemInterface.h"

using namespace DPsim;

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
};

void PyInterface::dealloc(PyInterface* self) {
	if (self->intf)
		delete self->intf;
	Py_TYPE(self)->tp_free((PyObject*) self);
}

PyObject* pyShmemInterface(PyObject *self, PyObject *args, PyObject *kwds) {
	static char *kwlist[] = {"queuelen", "samplelen", "polling", nullptr};
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
