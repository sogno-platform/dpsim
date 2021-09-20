/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <dpsim/Config.h>
#include <dpsim/Python/SystemTopology.h>
#include <dpsim/Simulation.h>
#include <dpsim/Python/EventChannel.h>
#include <dpsim/Timer.h>

namespace DPsim {
namespace Python {

	struct Simulation {
		PyObject_HEAD

		enum class State : int {
			stopped = 0,
			starting,
			running,
			pausing,
			paused,
			resuming,
			stopping,
			failed,
			overrun,
			done
		};

		DPsim::Simulation::Ptr sim;
		Python::SystemTopology *pySys;
		Python::EventChannel *channel;

		std::condition_variable *cond;
		std::mutex *mut;
		std::thread *thread;

		std::atomic<State> state;

		// Only relevant for real-time simulations
		double realTimeStep; /// effective timestep for real-time simulation
		bool realTime;
		bool startSync;
		bool failOnOverrun;
		bool singleStepping; /// Debugger like stepping for simulations

		Timer::StartTimePoint startTime;

		// List of additional objects that aren't directly used from Simulation
		// methods, but that a reference has be kept to to avoid them from being
		// freed.
		std::vector<PyObject*> refs;

		/// Function executed by the simulation thread
		static void threadFunction(Simulation* self);

		static void newState(Python::Simulation *self, Simulation::State newState);

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static int init(Simulation *self, PyObject *args, PyObject *kwds);
		static void dealloc(Simulation *self);

		// Methods that are actually available from Python
		static PyObject* addLogger(Simulation* self, PyObject* args, PyObject *kwargs);
		static PyObject* addEvent(Simulation* self, PyObject* args);
		static PyObject* pause(Simulation *self, PyObject *args);
		static PyObject* start(Simulation *self, PyObject *args);
		static PyObject* step(Simulation *self, PyObject *args);
		static PyObject* stop(Simulation *self, PyObject *args);
		static PyObject* reset(Simulation *self, PyObject *args);
		static PyObject* addEventFD(Simulation *self, PyObject *args);
		static PyObject* removeEventFD(Simulation *self, PyObject *args);
		static PyObject* setScheduler(Simulation *self, PyObject *args, PyObject *kwargs);

		// Setters
		static int setFinalTime(Simulation *self, PyObject *val, void *ctx);

		// Getters
		static PyObject* getState(Simulation *self, void *ctx);
		static PyObject* name(Simulation *self, void *ctx);
		static PyObject* steps(Simulation *self, void *ctx);
		static PyObject* time(Simulation *self, void *ctx);
		static PyObject* finalTime(Simulation *self, void *ctx);
		static PyObject* avgStepTime(Simulation *self, void *ctx);

		static const char *doc;
		static const char *docStart;
		static const char *docPause;
		static const char *docStop;
		static const char *docReset;
		static const char *docStep;
		static const char *docAddEvent;
		static const char *docAddLogger;
		static const char *docAddEventFD;
		static const char *docRemoveEventFD;
		static const char *docSetScheduler;
		static const char *docState;
		static const char *docName;
		static PyMethodDef methods[];
		static PyGetSetDef getset[];
		static PyTypeObject type;

#ifdef WITH_GRAPHVIZ
		static const char *docReprSVG;
		static PyObject* reprSVG(Simulation* self, PyObject* args);
#endif
	};
}
}
