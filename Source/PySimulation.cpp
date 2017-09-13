#include "PySimulation.h"

using namespace DPsim;

void PySimulation::simThreadFunction(PySimulation* pySim) {
	bool notDone = true;

	std::unique_lock<std::mutex> lk(*pySim->mut, std::defer_lock);
	pySim->numStep = 0;
	while (pySim->running && notDone) {
		notDone = pySim->sim->step(*pySim->log);
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
	// TODO: actually parse arguments (frequency, timestep etc.)
	self->log = new Logger("log.txt");
	self->sim = new Simulation(components, 2*PI*50, 1e-3, 0.3, *self->log);
	return 0;
};

void PySimulation::dealloc(PySimulation* self) {
	if (self->simThread) {
		// We have to cancel the running thread here, because otherwise self can't
		// be freed.
		PySimulation::stop((PyObject*)self, NULL);
		delete self->simThread;
	}
	if (self->sim)
		delete self->sim;
	delete self->mut;
	delete self->cond;
	Py_TYPE(self)->tp_free((PyObject*)self);
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
