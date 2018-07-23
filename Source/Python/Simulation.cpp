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
#include "Python/Simulation.h"

#include "Python/Interface.h"
#include "Python/Component.h"

#include <cfloat>
#include <iostream>

#ifdef WITH_RT
  #include <sys/timerfd.h>
  #include <time.h>
  #include <unistd.h>
#endif /* WITH_RT */

using namespace DPsim;

void DPsim::Python::Simulation::simThreadFunction(Python::Simulation* pySim)
{
#ifdef WITH_RT
	if (pySim->rt) {
		simThreadFunctionRT(pySim);
	}
#endif

	if (!pySim->rt) {
		simThreadFunctionNonRT(pySim);
	}
}

void DPsim::Python::Simulation::simThreadFunctionNonRT(DPsim::Python::Simulation *pySim)
{
	Real time, endTime;
	std::unique_lock<std::mutex> lk(*pySim->mut, std::defer_lock);

	endTime = pySim->sim->getFinalTime();
#ifdef __linux__
	pySim->sim->sendNotification(DPsim::Simulation::Event::Started);
#endif
	pySim->numStep = 0;
	while (pySim->running && time < endTime) {
		time = pySim->sim->step();

		pySim->numStep++;

		if (pySim->sigPause) {
			lk.lock();
			pySim->state = State::Paused;
#ifdef __linux__
			pySim->sim->sendNotification(DPsim::Simulation::Event::Paused);
#endif
			pySim->cond->notify_one();
			pySim->cond->wait(lk);

			pySim->state = State::Running;
#ifdef __linux__
			pySim->sim->sendNotification(DPsim::Simulation::Event::Resumed);
#endif
			lk.unlock();
		}
	}
	lk.lock();
#ifdef __linux__
	pySim->sim->sendNotification(DPsim::Simulation::Event::Finished);
#endif
	pySim->state = State::Done;
	pySim->cond->notify_one();
}

#ifdef WITH_RT
void DPsim::Python::Simulation::simThreadFunctionRT(Python::Simulation *pySim)
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

// TODO
//	ts.it_value.tv_sec = (time_t) pySim->sim->getTimeStep();
//	ts.it_value.tv_nsec = (long) (pySim->sim->getTimeStep() *1e9);
	ts.it_interval = ts.it_value;

	// optional start synchronization
	if (pySim->startSync) {
// TODO
//		pySim->sim->step(false); // first step, sending the initial values
//		pySim->sim->step(true); // blocking step for synchronization + receiving the initial state of the other network
//		pySim->sim->increaseByTimeStep();
	}

	if (timerfd_settime(timerfd, 0, &ts, 0) < 0) {
		throw SystemError("Failed to arm timerfd");
	}

	while (pySim->running && notDone) {
// TODO
//		notDone = pySim->sim->step();
		if (read(timerfd, timebuf, 8) < 0) {
			throw SystemError("Read from timerfd failed");
		}

		overrun = *((uint64_t*) timebuf);
		if (overrun > 1) {
// TODO
//			std::cerr << "timerfd overrun of " << overrun-1 << " at " << pySim->sim->getTime() << std::endl;
		}

		pySim->numStep++;
// TODO
//		pySim->sim->increaseByTimeStep();

		// in case it wasn't obvious, pausing a RT simulation is a bad idea
		// as it will most likely lead to overruns, but it's possible nonetheless
		if (pySim->sigPause) {
			lk.lock();
			pySim->state = State::Paused;
			pySim->cond->notify_one();
			pySim->cond->wait(lk);
			pySim->state = State::Running;
			lk.unlock();
		}
	}

	close(timerfd);
	lk.lock();

	pySim->state = State::Done;
	pySim->cond->notify_one();
}
#endif /* WITH_RT */

PyObject* DPsim::Python::Simulation::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds)
{
	Python::Simulation *self;

	self = (Python::Simulation*) type->tp_alloc(type, 0);
	if (self) {
		// since mutex, thread etc. have no copy-constructor, but we can't use
		// our own C++ constructor that could be called from python, we need to
		// implement them as pointers
		self->cond = new std::condition_variable();
		self->mut = new std::mutex();

		using SharedSimPtr = std::shared_ptr<DPsim::Simulation>;
		using PyObjectsList = std::vector<PyObject *>;

		new (&self->sim) SharedSimPtr();
		new (&self->refs) PyObjectsList();
	}

	return (PyObject*) self;
}

int DPsim::Python::Simulation::init(Python::Simulation* self, PyObject *args, PyObject *kwds)
{
	static char *kwlist[] = {"name", "system", "timestep", "duration", "rt", "start_sync", "sim_type", "solver_type", NULL};
	double timestep = 1e-3, duration = DBL_MAX;
	const char *name = nullptr;
	int t = 0, s = 0;

	enum Solver::Type solverType;
	enum Domain domain;

	self->rt = 0;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|ddbbii", kwlist,
		&name, &self->pySys, &timestep, &duration, &self->rt, &self->startSync, &s, &t)) {
		return -1;
	}

	switch (s) {
		case 0: domain = CPS::Domain::DP; break;
		case 1: domain = CPS::Domain::EMT; break;
		default:
			PyErr_SetString(PyExc_TypeError, "Invalid sim_type argument (must be 0 or 1)");
			return -1;
	}

	switch (t) {
		case 0: solverType = DPsim::Solver::Type::MNA; break;
		case 1: solverType = DPsim::Solver::Type::IDA; break;
		default:
			PyErr_SetString(PyExc_TypeError, "Invalid solver_type argument (must be 0 or 1)");
			return -1;
	}

	if (!PyObject_TypeCheck(self->pySys, &CPS::Python::SystemTopologyType)) {
		PyErr_SetString(PyExc_TypeError, "Argument system must be dpsim.SystemTopology");
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

	Py_INCREF(self->pySys);

	self->sim = std::make_shared<DPsim::Simulation>(name, *self->pySys->sys, timestep, duration, domain, solverType, Logger::Level::INFO);
	return 0;
};

void DPsim::Python::Simulation::dealloc(Python::Simulation* self)
{
	if (self->simThread) {
		// We have to cancel the running thread here, because otherwise self can't
		// be freed.
		Python::Simulation::stop((PyObject*) self, NULL);
		self->simThread->join();
		delete self->simThread;
	}

	if (self->sim) {
		using SimSharedPtr = std::shared_ptr<DPsim::Simulation>;
		self->sim.~SimSharedPtr();
	}

	delete self->mut;
	delete self->cond;

	for (auto it : self->refs) {
		Py_DECREF(it);
	}

	// Since this is not a C++ destructor which would automatically call the
	// destructor of its members, we have to manually call the destructor of
	// the vectors here to free the associated memory.

	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using PyObjectsList = std::vector<PyObject *>;

	self->refs.~PyObjectsList();

	Py_XDECREF(self->pySys);
	Py_TYPE(self)->tp_free((PyObject*) self);
}

static const char* DocSimulationAddInterface =
"add_interface(intf)\n"
"Add an external interface to the simulation. "
"Before each timestep, values are read from this interface and results are written to this interface afterwards. "
"See the documentation of `Interface` for more details.\n"
"\n"
":param intf: The `Interface` to be added.";
PyObject* DPsim::Python::Simulation::addInterface(PyObject* self, PyObject* args)
{
#ifdef WITH_SHMEM
	DPsim::Python::Simulation *pySim = (DPsim::Python::Simulation*) self;
	PyObject* pyObj;
	CPS::Python::Interface* pyIntf;

	if (!PyArg_ParseTuple(args, "O", &pyObj))
		return nullptr;

	if (!PyObject_TypeCheck(pyObj, &CPS::Python::InterfaceType)) {
		PyErr_SetString(PyExc_TypeError, "Argument must be dpsim.Interface");
		return nullptr;
	}

	pyIntf = (CPS::Python::Interface*) pyObj;
	pySim->sim->addInterface(pyIntf->intf.get());
	Py_INCREF(pyObj);

	pySim->refs.push_back(pyObj);
	Py_INCREF(Py_None);

	return Py_None;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}

#if 0
static const char* DocSimulationLvector =
"lvector()\n"
"Return the left-side vector of the last step as a list of floats.";
PyObject*DPsim::Python::Simulation::lvector(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;

	if (pySim->state == State::Running) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}

	Matrix& lvector = pySim->sim->getLeftSideVector();
	PyObject* list = PyList_New(lvector.rows());

	for (int i = 0; i < lvector.rows(); i++)
		PyList_SetItem(list, i, PyFloat_FromDouble(lvector(i, 0)));

	return list;
}
#endif

static const char* DocSimulationPause =
"pause()\n"
"Pause the simulation at the next possible time (usually, after finishing the current timestep).\n"
"\n"
":raises: ``SystemError`` if the simulation is not running.\n";
PyObject* DPsim::Python::Simulation::pause(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	if (pySim->state != State::Running) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}

	pySim->sigPause = 1;
	pySim->cond->notify_one();

	while (pySim->state == State::Running)
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
PyObject* DPsim::Python::Simulation::start(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	if (pySim->state == State::Running) {
		PyErr_SetString(PyExc_SystemError, "Simulation already started");
		return nullptr;
	}
	else if (pySim->state == State::Done) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	}
	else if (pySim->state == State::Paused) {
		pySim->sigPause = 0;
		pySim->cond->notify_one();
	}
	else {
		pySim->sigPause = 0;
		pySim->state = State::Running;
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
PyObject* DPsim::Python::Simulation::step(PyObject *self, PyObject *args)
{
	Python::Simulation *pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	int oldStep = pySim->numStep;
	if (pySim->state == State::Stopped) {
		pySim->state = State::Running;
		pySim->sigPause = 1;
		pySim->running = true;
		pySim->simThread = new std::thread(simThreadFunction, pySim);
	}
	else if (pySim->state == State::Paused) {
		pySim->sigPause = 1;
		pySim->cond->notify_one();
	}
	else if (pySim->state == State::Done) {
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
PyObject* DPsim::Python::Simulation::stop(PyObject *self, PyObject *args)
{
	Python::Simulation* pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	pySim->running = false;

	while (pySim->state == State::Running)
		pySim->cond->wait(lk);

	Py_INCREF(Py_None);

	return Py_None;
}

static const char* DocSimulationWait =
"wait()\n"
"Block until the simulation is finished, returning immediately if this is already the case.\n"
"\n"
":raises: ``SystemError`` if the simulation is paused or was not started yet.";
PyObject* DPsim::Python::Simulation::wait(PyObject *self, PyObject *args)
{
	DPsim::Python::Simulation *pySim = (DPsim::Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	if (pySim->state == State::Done) {
		Py_INCREF(Py_None);
		return Py_None;
	}
	else if (pySim->state == State::Stopped) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}
	else if (pySim->state == State::Paused) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently paused");
		return nullptr;
	}

	while (pySim->state == State::Running)
		pySim->cond->wait(lk);

	Py_INCREF(Py_None);

	return Py_None;
}

#ifdef __linux__
static char* DocSimulationGetEventFD =
"get_eventfd(flags)\n"
"Return a poll()/select()'able file descriptor which can be used to asynchronously\n"
"notify the Python code about state changes and other events of the simulation.\n"
"\n"
":param flags: An optional mask of events which should be reported.\n"
":param coalesce: Do not report each event  but do a rate reduction instead.\n";
PyObject * DPsim::Python::Simulation::getEventFD(PyObject *self, PyObject *args) {
	int flags = -1, coalesce = 1, fd;
	Python::Simulation *pySim = (Python::Simulation *) self;

	if (!PyArg_ParseTuple(args, "|ii", &flags, &coalesce))
		return nullptr;

	fd = pySim->sim->getEventFD(flags, coalesce);
	if (fd < 0) {
		PyErr_SetString(PyExc_SystemError, "Failed to created event file descriptor");
		return nullptr;
	}

	return Py_BuildValue("i", fd);
}
#endif

static char* DocSimulationGetState =
"state\n"
"The current state of simulation.\n";
PyObject* DPsim::Python::Simulation::getState(PyObject *self, void *ctx)
{
	Python::Simulation* pySim = (Python::Simulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);

	return Py_BuildValue("i", pySim->state);
}

static char* DocSimulationGetName =
"name\n"
"The name of the simulation.";
PyObject* DPsim::Python::Simulation::getName(PyObject *self, void *ctx)
{
	Python::Simulation *pySim = (Python::Simulation*) self;

	return PyUnicode_FromString(pySim->sim->getName().c_str());
}

PyObject* DPsim::Python::Simulation::getSteps(PyObject *self, void *ctx)
{
	Python::Simulation *pySim = (Python::Simulation*) self;

	return Py_BuildValue("i", pySim->sim->getTimeStepCount());
}

PyObject* DPsim::Python::Simulation::getTime(PyObject *self, void *ctx)
{
	Python::Simulation *pySim = (Python::Simulation*) self;

	return Py_BuildValue("f", pySim->sim->getTime());
}

PyObject* DPsim::Python::Simulation::getFinalTime(PyObject *self, void *ctx)
{
	Python::Simulation *pySim = (Python::Simulation*) self;

	return Py_BuildValue("f", pySim->sim->getFinalTime());
}


static PyGetSetDef Simulation_attrs[] = {
	{"state", DPsim::Python::Simulation::getState, NULL, DocSimulationGetState, NULL},
	{"name",  DPsim::Python::Simulation::getName, NULL, DocSimulationGetName, NULL},
	{"steps",  DPsim::Python::Simulation::getSteps, NULL, NULL, NULL},
	{"time",  DPsim::Python::Simulation::getTime, NULL, NULL, NULL},
	{"final_time",  DPsim::Python::Simulation::getFinalTime, NULL, NULL, NULL},
	{NULL, NULL, NULL, NULL, NULL}
};

static PyMethodDef Simulation_methods[] = {
	{"add_interface",	DPsim::Python::Simulation::addInterface, METH_VARARGS, DocSimulationAddInterface},
	{"pause",		DPsim::Python::Simulation::pause, METH_NOARGS, DocSimulationPause},
	{"start",		DPsim::Python::Simulation::start, METH_NOARGS, DocSimulationStart},
	{"step",		DPsim::Python::Simulation::step, METH_NOARGS, DocSimulationStep},
	{"stop",		DPsim::Python::Simulation::stop, METH_NOARGS, DocSimulationStop},
	{"wait",		DPsim::Python::Simulation::wait, METH_NOARGS, DocSimulationWait},
#ifdef __linux__
	{"get_eventfd",         DPsim::Python::Simulation::getEventFD, METH_VARARGS, DocSimulationGetEventFD},
#endif
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
	sizeof(DPsim::Python::Simulation),       /* tp_basicsize */
	0,                                       /* tp_itemsize */
	(destructor)DPsim::Python::Simulation::dealloc, /* tp_dealloc */
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
	Simulation_attrs,                        /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc)DPsim::Python::Simulation::init, /* tp_init */
	0,                                       /* tp_alloc */
	DPsim::Python::Simulation::newfunc,      /* tp_new */
};
