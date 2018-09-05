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

#include <dpsim/Config.h>
#include <dpsim/Timer.h>
#include <dpsim/Simulation.h>
#include <dpsim/Python/SystemTopology.h>
#include <dpsim/Python/EventChannel.h>

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
		bool singleStepping; /// Debugger like stepping for simulations

		Timer::StartTimePoint startTime;

		// List of additional objects that aren't directly used from Simulation
		// methods, but that a reference has be kept to to avoid them from being
		// freed (e.g. Interfaces).
		std::vector<PyObject*> refs;

		// Function executed by the simulation thread
		static void threadFunction(Simulation* self);

		static void newState(Python::Simulation *self, Simulation::State newState);

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static int init(Simulation* self, PyObject *args, PyObject *kwds);
		static void dealloc(Simulation*);

		// Methods that are actually available from Python
		static PyObject* addInterface(Simulation *self, PyObject *args);
		static PyObject* pause(Simulation *self, PyObject *args);
		static PyObject* start(Simulation *self, PyObject *args);
		static PyObject* step(Simulation *self, PyObject *args);
		static PyObject* stop(Simulation *self, PyObject *args);
		static PyObject* wait(Simulation *self, PyObject *args);
		static PyObject* getEventFD(Simulation *self, PyObject *args);

		// Getters
		static PyObject* getState(Simulation *self, void *ctx);
		static PyObject* name(Simulation *self, void *ctx);
		static PyObject* steps(Simulation *self, void *ctx);
		static PyObject* time(Simulation *self, void *ctx);
		static PyObject* finalTime(Simulation *self, void *ctx);
	};

	extern PyTypeObject SimulationType;
}
}
