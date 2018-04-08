/** Python simulation
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "Config.h"
#include "../Simulation.h"
#include "cps/Source/Component.h"

namespace DPsim {

	enum SimState {
		StateStopped = 0,
		StateRunning,
		StatePaused,
		StateDone
	};

namespace Python {

	struct Simulation {
		PyObject_HEAD

		DPsim::Simulation *sim;
		Logger *log, *llog, *rlog;

		std::condition_variable *cond;
		std::mutex *mut;
		std::atomic_bool running;
		std::atomic_int sigPause, numStep;
		std::thread *simThread;
		SimState state;

		bool rt;
		bool startSync;

		PyObject* pyComps; // Components as a (Python) list of PyComponents
		CPS::Component::List comps;
		int numSwitch;

		// List of additional objects that aren't directly used from Simulation
		// methods, but that a reference has be kept to to avoid them from being
		// freed (e.g. ExternalInterfaces).
		std::vector<PyObject*> refs;

		// Function executed by the simulation thread
		static void simThreadFunction(Simulation* pySim);
#ifdef WITH_RT
		static void simThreadFunctionRT(Simulation* pySim);
#endif

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static int init(Simulation* self, PyObject *args, PyObject *kwds);
		static void dealloc(Simulation*);

		// Methods that are actually available from Python
		static PyObject* addInterface(PyObject *self, PyObject *args);
		static PyObject* lvector(PyObject *self, PyObject *args);
		static PyObject* name(PyObject *self, PyObject *args);
		static PyObject* pause(PyObject *self, PyObject *args);
		static PyObject* start(PyObject *self, PyObject *args);
		static PyObject* step(PyObject *self, PyObject *args);
		static PyObject* stop(PyObject *self, PyObject *args);
		static PyObject* updateMatrix(PyObject *self, PyObject *args);
		static PyObject* wait(PyObject *self, PyObject *args);
	};

	extern PyTypeObject SimulationType;
}
}
