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

#include "Config.h"

#include "Python/Component.h"
#include "Python/Interface.h"
#include "Python/Simulation.h"

#include "../SystemModel.h"

#include <cfloat>
#include <iostream>

#ifdef WITH_RT
  #include <sys/timerfd.h>
  #include <time.h>
  #include <unistd.h>
#endif /* WITH_RT */

using namespace DPsim;

void Python::Simulation::simThreadFunction(Python::Simulation* pySim)
{
	bool notDone = true;

#ifdef WITH_RT
	if (pySim->rt) {
		simThreadFunctionRT(pySim);
	}
#endif

	if (!pySim->rt) {
		simThreadFunctionNonRT(pySim);
	}
}

void Python::Simulation::simThreadFunctionNonRT(Python::Simulation *pySim)
{
	std::unique_lock<std::mutex> lk(*pySim->mut, std::defer_lock);

	pySim->numStep = 0;
	while (pySim->running && notDone) {
		notDone = pySim->sim->step();

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

#ifdef WITH_RT
void Python::Simulation::simThreadFunctionRT(Python::Simulation *pySim)
{
	bool notDone = true;
	char timebuf[8];
	int timerfd;
	struct itimerspec ts;
	uint64_t overrun;

	std::unique_lock<std::mutex> lk(*pySim->mut, std::defer_lock);

	pySim->numStep = 0;

	// RT method is limited to timerfd right now; to implement it with Exceptions,
	// look at Simulation::runRT
	timerfd = timerfd_create(CLOCK_REALTIME, 0);

	// TODO: better error mechanism (somehow pass an Exception to the Python thread?)
	if (timerfd < 0) {
		throw SystemError("Failed to create timerfd");
	}

	ts.it_value.tv_sec = (time_t) pySim->sim->getTimeStep();
	ts.it_value.tv_nsec = (long) (pySim->sim->getTimeStep() *1e9);
	ts.it_interval = ts.it_value;

	// optional start synchronization
	if (pySim->startSync) {
		pySim->sim->step(false); // first step, sending the initial values
		pySim->sim->step(true); // blocking step for synchronization + receiving the initial state of the other network
		pySim->sim->increaseByTimeStep();
	}

	if (timerfd_settime(timerfd, 0, &ts, 0) < 0) {
		throw SystemError("Failed to arm timerfd");
	}

	while (pySim->running && notDone) {
		notDone = pySim->sim->step();
		if (read(timerfd, timebuf, 8) < 0) {
			throw SystemError("Read from timerfd failed");
		}

		overrun = *((uint64_t*) timebuf);
		if (overrun > 1) {
			std::cerr << "timerfd overrun of " << overrun-1 << " at " << pySim->sim->getTime() << std::endl;
		}

		pySim->numStep++;
		pySim->sim->increaseByTimeStep();

		// in case it wasn't obvious, pausing a RT simulation is a bad idea
		// as it will most likely lead to overruns, but it's possible nonetheless
		if (pySim->sigPause) {
			lk.lock();
			pySim->state = StatePaused;
			pySim->cond->notify_one();
			pySim->cond->wait(lk);
			pySim->state = StateRunning;
			lk.unlock();
		}
	}

	close(timerfd);
	lk.lock();

	pySim->state = StateDone;
	pySim->cond->notify_one();
}
#endif /* WITH_RT */

PyObject* Python::Simulation::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds)
{
	Python::Simulation *self;

	self = (Python::Simulation*) type->tp_alloc(type, 0);
	if (self) {
		// since mutex, thread etc. have no copy-constructor, but we can't use
		// our own C++ constructor that could be called from python, we need to
		// implement them as pointers
		self->cond = new std::condition_variable();
		self->mut = new std::mutex();
		self->numSwitch = 0;
	}
	return (PyObject*) self;
}

int Python::Simulation::init(Python::Simulation* self, PyObject *args, PyObject *kwds)
{
	static char *kwlist[] = {"name", "components", "frequency", "timestep", "duration", "rt", "start_sync", "type", NULL};
	double frequency = 50, timestep = 1e-3, duration = DBL_MAX;
	const char *name = nullptr;
	int type = 0;
	enum SimulationType simType;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|dddbbi", kwlist,
		&name, &self->pyComps, &frequency, &timestep, &duration, &self->rt, &self->startSync, &type)) {
		return -1;
	}

	switch (type) {
		case 0: simType = DPsim::SimulationType::DP; break;
		case 1: simType = DPsim::SimulationType::EMT; break;
		default:
			PyErr_SetString(PyExc_TypeError, "Invalid type argument (must be 0 or 1)");
			return -1;
	}

	if (!compsFromPython(self->pyComps, self->comps)) {
		PyErr_SetString(PyExc_TypeError, "Invalid components argument (must by list of dpsim.Component)");
		return -1;
	}

#ifndef WITH_RT
	if (self->rt) {
		PyErr_SetString(PyExc_SystemError, "RT mode not available on this platform");
		return -1;
	}
#endif /* WITH_RT */

	if (self->startSync && !self->rt) {
		PyErr_Format(PyExc_ValueError, "start_sync only valid in rt mode");
		return -1;
	}

	Py_INCREF(self->pyComps);

	self->sim = new DPsim::Simulation(name, self->comps, 2*PI*frequency, timestep, duration, Logger::Level::INFO, simType);
	return 0;
};

void Python::Simulation::dealloc(Python::Simulation* self)
{
	if (self->simThread) {
		// We have to cancel the running thread here, because otherwise self can't
		// be freed.
		Python::Simulation::stop((PyObject*)self, NULL);
		self->simThread->join();
		delete self->simThread;
	}

	if (self->sim)
		delete self->sim;

	if (self->log)
		delete self->log;

	if (self->llog)
		delete self->llog;

	if (self->rlog)
		delete self->rlog;

	delete self->mut;
	delete self->cond;

	for (auto it : self->refs) {
		Py_DECREF(it);
	}

	// Since this is not a C++ destructor which would automatically call the
	// destructor of its members, we have to manually call the destructor of
	// the vectors here to free the associated memory.

	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using ComponentList = DPsim::Component::List;
	using PyObjectsList = std::vector<PyObject *>;

	self->comps.~ComponentList();
	self->refs.~PyObjectsList();

	Py_XDECREF(self->pyComps);
	Py_TYPE(self)->tp_free((PyObject*) self);
}

static const char* DocSimulationAddInterface =
"add_interface(intf)\n"
"Add an external interface to the simulation. "
"Before each timestep, values are read from this interface and results are written to this interface afterwards. "
"See the documentation of `Interface` for more details.\n"
"\n"
":param intf: The `Interface` to be added.";
PyObject* Python::Simulation::addInterface(PyObject* self, PyObject* args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
	PyObject* pyObj;
	Python::Interface* pyIntf;

	if (!PyArg_ParseTuple(args, "O", &pyObj))
		return nullptr;

	if (!PyObject_TypeCheck(pyObj, &Python::InterfaceType)) {
		PyErr_SetString(PyExc_TypeError, "Argument must be dpsim.Interface");
		return nullptr;
	}

	pyIntf = (Interface*) pyObj;
	pySim->sim->addExternalInterface(pyIntf->intf);
	Py_INCREF(pyObj);

	pySim->refs.push_back(pyObj);
	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocSimulationLvector =
"lvector()\n"
"Return the left-side vector of the last step as a list of floats.";
PyObject* Python::Simulation::lvector(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;

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

static const char* DocSimulationName =
"name()\n"
"Return the of the simulation.";
PyObject* Python::Simulation::name(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;

	return PyUnicode_FromString(pySim->sim->getName().c_str());
}

static const char* DocSimulationPause =
"pause()\n"
"Pause the simulation at the next possible time (usually, after finishing the current timestep).\n"
"\n"
":raises: ``SystemError`` if the simulation is not running.\n";
PyObject* Python::Simulation::pause(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
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

static const char* DocSimulationStart =
"start()\n"
"Start the simulation, or resume it if it has been paused. "
"The simulation runs in a separate thread, so this method doesn't wait for the "
"simulation to finish, but returns immediately.\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* Python::Simulation::start(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	if (pySim->state == StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation already started");
		return nullptr;
	}
	else if (pySim->state == StateDone) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	}
	else if (pySim->state == StatePaused) {
		pySim->sigPause = 0;
		pySim->cond->notify_one();
	}
	else {
		pySim->sigPause = 0;
		pySim->state = StateRunning;
		pySim->running = true;
		pySim->simThread = new std::thread(simThreadFunction, pySim);
	}

	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocSimulationStep =
"step()\n"
"Perform a single step of the simulation (possibly the first).\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* Python::Simulation::step(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	int oldStep = pySim->numStep;
	if (pySim->state == StateStopped) {
		pySim->state = StateRunning;
		pySim->sigPause = 1;
		pySim->running = true;
		pySim->simThread = new std::thread(simThreadFunction, pySim);
	}
	else if (pySim->state == StatePaused) {
		pySim->sigPause = 1;
		pySim->cond->notify_one();
	}
	else if (pySim->state == StateDone) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	}
	else {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}

	while (pySim->numStep == oldStep)
		pySim->cond->wait(lk);

	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocSimulationStop =
"stop()\n"
"Stop the simulation at the next possible time. The simulation thread is canceled "
"and the simulation can not be restarted. No-op if the simulation is not running.";
PyObject* Python::Simulation::stop(PyObject *self, PyObject *args)
{
	Python::Simulation* pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	pySim->running = false;

	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);

	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocSimulationWait =
"wait()\n"
"Block until the simulation is finished, returning immediately if this is already the case.\n"
"\n"
":raises: ``SystemError`` if the simulation is paused or was not started yet.";
PyObject* Python::Simulation::wait(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	if (pySim->state == StateDone) {
		Py_INCREF(Py_None);
		return Py_None;
	}
	else if (pySim->state == StateStopped) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}
	else if (pySim->state == StatePaused) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently paused");
		return nullptr;
	}

	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);

	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocSimulationGetState =
"get_state()\n"
"Get current state of simulation.\n";
PyObject* Python::Simulation::getState(PyObject *self, PyObject *args)
{
	Python::Simulation* pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	/** @todo Create and return state enum */
}

static PyMethodDef Simulation_methods[] = {
	{"add_interface", Python::Simulation::addInterface, METH_VARARGS, DocSimulationAddInterface},
	{"get_state", Python::Simulation::getState, METH_NOARGS, DocSimulationGetState},
	{"lvector", Python::Simulation::lvector, METH_NOARGS, DocSimulationLvector},
	{"name", Python::Simulation::name, METH_NOARGS, DocSimulationName},
	{"pause", Python::Simulation::pause, METH_NOARGS, DocSimulationPause},
	{"start", Python::Simulation::start, METH_NOARGS, DocSimulationStart},
	{"step", Python::Simulation::step, METH_NOARGS, DocSimulationStep},
	{"stop", Python::Simulation::stop, METH_NOARGS, DocSimulationStop},
	{"wait", Python::Simulation::wait, METH_NOARGS, DocSimulationWait},
	{NULL, NULL, 0, NULL}
};

static const char* DocSimulation =
"A single simulation.\n"
"\n"
"Proper ``__init__`` signature:\n"
"\n"
"``__init__(self, name, components, frequency=50.0, timestep=1e-3, duration=sys.float_info.max, "
"rt=false, start_sync=false)``.\n\n"
"``name`` is the unique name of the simulation which is used to create the log files.\n\n"
"``components`` must be a list of `Component` that are to be simulated.\n\n"
"``frequency`` is the nominal system frequency in Hz.\n\n"
"``timestep`` is the simulation timestep in seconds.\n\n"
"``duration`` is the duration after which the simulation stops; the default value "
"lets the simulation run indefinitely until being stopped manually by `stop`.\n\n"
"If ``rt`` is True, the simulation will run in realtime mode. The simulation will "
"try to match simulation time with the wall clock time; violations will be logged.\n\n"
"If ``start_sync`` is given as well, a specific method for synchronizing the "
"simulation start with other external simulators will be used. After performing "
"a first step with the initial values, the simulation will wait until receiving "
"the first message(s) from the external interface(s) until the realtime simulation "
"starts properly.";
PyTypeObject DPsim::Python::SimulationType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"dpsim.Simulation",                      /* tp_name */
	sizeof(Python::Simulation),              /* tp_basicsize */
	0,                                       /* tp_itemsize */
	(destructor)Python::Simulation::dealloc, /* tp_dealloc */
	0,                                       /* tp_print */
	0,                                       /* tp_getattr */
	0,                                       /* tp_setattr */
	0,                                       /* tp_reserved */
	0,                                       /* tp_repr */
	0,                                       /* tp_as_number */
	0,                                       /* tp_as_sequence */
	0,                                       /* tp_as_mapping */
	0,                                       /* tp_hash  */
	0,                                       /* tp_call */
	0,                                       /* tp_str */
	0,                                       /* tp_getattro */
	0,                                       /* tp_setattro */
	0,                                       /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,/* tp_flags */
	DocSimulation,                           /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Simulation_methods,                      /* tp_methods */
	0,                                       /* tp_members */
	0,                                       /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc)Python::Simulation::init,      /* tp_init */
	0,                                       /* tp_alloc */
	Python::Simulation::newfunc,             /* tp_new */
};
