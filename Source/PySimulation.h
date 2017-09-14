#ifndef PYSIMULATION_H
#define PYSIMULATION_H

#include <Python.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "Components/BaseComponent.h"
#include "Simulation.h"

namespace DPsim {
	enum SimState {
		StateStopped = 0,
		StateRunning,
		StatePaused,
		StateDone
	};

	struct PySimulation {
		PyObject_HEAD

		Simulation *sim;
		Logger *log; // TODO other loggers

		std::condition_variable *cond;
		std::mutex *mut;
		std::atomic_bool running;
		std::atomic_int sigPause, numStep;
		std::thread *simThread;
		SimState state;
		
		// Function executed by the simulation thread
		static void simThreadFunction(PySimulation* pySim);

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static int init(PySimulation* self, PyObject *args, PyObject *kwds);
		static void dealloc(PySimulation*);

		// Methods that are actually available from Python
		static PyObject* start(PyObject *self, PyObject *args);
		static PyObject* step(PyObject *self, PyObject *args);
		static PyObject* stop(PyObject *self, PyObject *args);
		static PyObject* pause(PyObject *self, PyObject *args);
		static PyObject* wait(PyObject *self, PyObject *args);
	};

	static PyMethodDef PySimulation_methods[] = {
		{"start", PySimulation::start, METH_NOARGS, "Start the simulation, or resume if it is paused."},
		{"step", PySimulation::step, METH_NOARGS, "Perform a single simulation step."},
		{"stop", PySimulation::stop, METH_NOARGS, "Cancel the running simulation."},
		{"pause", PySimulation::pause, METH_NOARGS, "Pause the already running simulation."},
		{"wait", PySimulation::wait, METH_NOARGS, "Wait for the simulation to finish."},
		{NULL, NULL, 0, NULL}
	};

	static PyTypeObject PySimulationType = {
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

	extern std::vector<BaseComponent*> components;
};
#endif
