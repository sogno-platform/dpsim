#include "PySimulation.h"
#include "PyComponent.h"

#include <cfloat>
#include <iostream>

using namespace DPsim;

static PyMethodDef PySimulation_methods[] = {
	{"lvector", PySimulation::lvector, METH_NOARGS, "Returns the left-side vector from the last step."},
	{"start", PySimulation::start, METH_NOARGS, "Start the simulation, or resume if it is paused."},
	{"step", PySimulation::step, METH_NOARGS, "Perform a single simulation step."},
	{"stop", PySimulation::stop, METH_NOARGS, "Cancel the running simulation."},
	{"pause", PySimulation::pause, METH_NOARGS, "Pause the already running simulation."},
	{"wait", PySimulation::wait, METH_NOARGS, "Wait for the simulation to finish."},
	{NULL, NULL, 0, NULL}
};

PyTypeObject DPsim::PySimulationType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"dpsim.Simulation",                /* tp_name */
	sizeof(PySimulation),              /* tp_basicsize */
	0,                                 /* tp_itemsize */
	(destructor)PySimulation::dealloc, /* tp_dealloc */
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
	"A single simulation.",            /* tp_doc */
	0,                                 /* tp_traverse */
	0,                                 /* tp_clear */
	0,                                 /* tp_richcompare */
	0,                                 /* tp_weaklistoffset */
	0,                                 /* tp_iter */
	0,                                 /* tp_iternext */
	PySimulation_methods,              /* tp_methods */
	0,                                 /* tp_members */
	0,                                 /* tp_getset */
	0,                                 /* tp_base */
	0,                                 /* tp_dict */
	0,                                 /* tp_descr_get */
	0,                                 /* tp_descr_set */
	0,                                 /* tp_dictoffset */
	(initproc)PySimulation::init,      /* tp_init */
	0,                                 /* tp_alloc */
	PySimulation::newfunc,             /* tp_new */
};

void PySimulation::simThreadFunction(PySimulation* pySim) {
	bool notDone = true;
	Logger rlog("rvector.csv"), llog("lvector.csv");

	std::unique_lock<std::mutex> lk(*pySim->mut, std::defer_lock);
	pySim->numStep = 0;
	while (pySim->running && notDone) {
		notDone = pySim->sim->step(*pySim->log, llog, rlog);
		pySim->numStep++;
		pySim->sim->increaseByTimeStep();
		if (pySim->sigPause) {
			lk.lock();
			pySim->state = StatePaused;
			pySim->cond->notify_one();
			pySim->cond->wait(lk);
			pySim->state = StateRunning;
			lk.unlock();
		}
	}
	lk.lock();
	pySim->state = StateDone;
	pySim->cond->notify_one();
}

PyObject* PySimulation::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds) {
	PySimulation *self;

	self = (PySimulation*) type->tp_alloc(type, 0);
	if (self) {
		// since mutex, thread etc. have no copy-constructor, but we can't use
		// our own C++ constructor that could be called from python, we need to
		// implement them as pointers
		self->cond = new std::condition_variable();
		self->mut = new std::mutex();
	}
	return (PyObject*) self;
}

int PySimulation::init(PySimulation* self, PyObject *args, PyObject *kwds) {
	static char *kwlist[] = {"components", "frequency", "timestep", "duration", "log", NULL};
	double frequency = 50, timestep = 1e-3, duration = DBL_MAX;
	const char *log = nullptr;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|ddds", kwlist,
		&self->pyComps, &frequency, &timestep, &duration, &log))
		return -1;
	if (!compsFromPython(self->pyComps, self->comps)) {
		PyErr_SetString(PyExc_TypeError, "Invalid components argument (must by list of dpsim.Component)");
		return -1;
	}
	Py_INCREF(self->pyComps);
	if (log)
		self->log = new Logger(log);
	else
		self->log = new Logger();
	self->sim = new Simulation(self->comps, 2*PI*frequency, timestep, duration, *self->log);
	return 0;
};

void PySimulation::dealloc(PySimulation* self) {
	if (self->simThread) {
		// We have to cancel the running thread here, because otherwise self can't
		// be freed.
		PySimulation::stop((PyObject*)self, NULL);
		self->simThread->join();
		delete self->simThread;
	}

	if (self->sim)
		delete self->sim;
	if (self->log)
		delete self->log;
	delete self->mut;
	delete self->cond;

	// Since this is not a C++ destructor which would automatically call the
	// destructor of its members, we have to manually call the destructor of
	// the component vector here to free the associated memory.
	self->comps.~vector<BaseComponent*>();

	Py_XDECREF(self->pyComps);
	Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* PySimulation::lvector(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	if (pySim->state == StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}
	Matrix& lvector = pySim->sim->getLeftSideVector();
	PyObject* list = PyList_New(lvector.rows());
	for (int i = 0; i < lvector.rows(); i++)
		PyList_SetItem(list, i, PyFloat_FromDouble(lvector(i, 0)));
	return list;
}

PyObject* PySimulation::start(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	if (pySim->state == StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation already started");
		return nullptr;
	} else if (pySim->state == StateDone) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	} else if (pySim->state == StatePaused) {
		pySim->sigPause = 0;
		pySim->cond->notify_one();
	} else {
		pySim->sigPause = 0;
		pySim->state = StateRunning;
		pySim->running = true;
		pySim->simThread = new std::thread(simThreadFunction, pySim);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* PySimulation::step(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	int oldStep = pySim->numStep;
	if (pySim->state == StateStopped) {
		pySim->state = StateRunning;
		pySim->sigPause = 1;
		pySim->running = true;
		pySim->simThread = new std::thread(simThreadFunction, pySim);
	} else if (pySim->state == StatePaused) {
		pySim->sigPause = 1;
		pySim->cond->notify_one();
	} else if (pySim->state == StateDone) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	} else {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}
	while (pySim->numStep == oldStep)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* PySimulation::stop(PyObject *self, PyObject *args) {
	PySimulation* pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	pySim->running = false;
	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* PySimulation::pause(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	if (pySim->state != StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}
	pySim->sigPause = 1;
	pySim->cond->notify_one();
	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

PyObject* PySimulation::wait(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	if (pySim->state == StateDone) {
		Py_INCREF(Py_None);
		return Py_None;
	} else if (pySim->state == StateStopped) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	} else if (pySim->state == StatePaused) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently paused");
		return nullptr;
	}
	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}
