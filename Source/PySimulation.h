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

		PyObject* pyComps; // Components as a (Python) list of PyComponents
		std::vector<BaseComponent*> comps;
		
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
		static PyObject* lvector(PyObject *self, PyObject *args);
		static PyObject* start(PyObject *self, PyObject *args);
		static PyObject* step(PyObject *self, PyObject *args);
		static PyObject* stop(PyObject *self, PyObject *args);
		static PyObject* pause(PyObject *self, PyObject *args);
		static PyObject* wait(PyObject *self, PyObject *args);
	};

	extern PyTypeObject PySimulationType;
};
#endif
