#include "PySimulation.h"

#include <cfloat>
#include <iostream>

using namespace DPsim;

std::vector<BaseComponent*> DPsim::components;

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
	static char *kwlist[] = {"frequency", "timestep", "duration", "log", NULL};
	double frequency = 50, timestep = 1e-3, duration = DBL_MAX;
	const char *log = nullptr;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "|ddds", kwlist,
		&frequency, &timestep, &duration, &log))
		return -1;
	if (log)
		self->log = new Logger(log);
	else
		self->log = new Logger();
	self->sim = new Simulation(components, 2*PI*frequency, timestep, duration, *self->log);
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
