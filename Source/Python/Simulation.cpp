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

#include <dpsim/Config.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim/Python/Simulation.h>
#include <dpsim/Python/Interface.h>
#include <dpsim/Python/Component.h>

#include <cfloat>
#include <iostream>

using namespace DPsim;
using namespace CPS;

void Python::Simulation::newState(Python::Simulation *self, Simulation::State newState)
{
	uint32_t evt = static_cast<uint32_t>(newState);

	self->channel->sendEvent(evt);
	self->state = newState;
}

void Python::Simulation::threadFunction(DPsim::Python::Simulation *self)
{
	Real time, endTime;
	std::unique_lock<std::mutex> lk(*self->mut, std::defer_lock);

	Timer timer;

	// optional start synchronization
	if (self->startSync) {
		self->sim->sync();
	}

	if (self->realTime) {
		timer.setInterval(self->sim->timeStep());
		timer.start();
	}

	endTime = self->sim->finalTime();

	newState(self, Simulation::State::running);

	while (time < endTime) {
		time = self->sim->step();

		if (self->realTime) {
			try {
				timer.sleep();
			}
			catch (Timer::OverrunException e) {
				newState(self, Simulation::State::overrun);
				break;
			}
		}

		if (self->state == State::pausing || self->singleStepping) {
			lk.lock();

			newState(self, Simulation::State::paused);
			self->cond->notify_one();

			while (self->state == State::paused)
				self->cond->wait(lk);

			newState(self, Simulation::State::running);

			lk.unlock();
		}

		if (self->state != State::stopping)
			break;
	}
	lk.lock();

	newState(self, Simulation::State::done);

	self->cond->notify_one();
}

PyObject* DPsim::Python::Simulation::newfunc(PyTypeObject* subtype, PyObject *args, PyObject *kwds)
{
	Python::Simulation *self;

	self = (Python::Simulation*) subtype->tp_alloc(subtype, 0);
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

		self->realTime = false;
		self->startSync = false;
	}

	return (PyObject*) self;
}

int DPsim::Python::Simulation::init(Simulation* self, PyObject *args, PyObject *kwds)
{
	static const char *kwlist[] = {"name", "system", "timestep", "duration", "sim_type", "solver_type", "rt", "start_sync", "single_stepping", nullptr};
	double timestep = 1e-3, duration = DBL_MAX;
	const char *name = nullptr;
	int t = 0, s = 0;
	int rt = 0, ss = 0, st = 0;

	enum Solver::Type solverType;
	enum Domain domain;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|ddiibbb", (char **) kwlist,
		&name, &self->pySys, &timestep, &duration, &s, &t, &rt, &ss, &st)) {
		return -1;
	}

	self->realTime = rt;
	self->startSync = ss;
	self->singleStepping = st;

	switch (s) {
		case 0: domain = CPS::Domain::DP; break;
		case 1: domain = CPS::Domain::EMT; break;
		default:
			PyErr_SetString(PyExc_TypeError, "Invalid sim_type argument (must be 0 or 1)");
			return -1;
	}

	switch (t) {
		case 0: solverType = DPsim::Solver::Type::MNA; break;
		case 1: solverType = DPsim::Solver::Type::DAE; break;
		default:
			PyErr_SetString(PyExc_TypeError, "Invalid solver_type argument (must be 0 or 1)");
			return -1;
	}

	if (!PyObject_TypeCheck(self->pySys, &DPsim::Python::SystemTopologyType)) {
		PyErr_SetString(PyExc_TypeError, "Argument system must be dpsim.SystemTopology");
		return -1;
	}

	Py_INCREF(self->pySys);

	if (self->realTime)
		self->sim = std::make_shared<DPsim::RealTimeSimulation>(name, *self->pySys->sys, timestep, duration, domain, solverType, Logger::Level::INFO);
	else
		self->sim = std::make_shared<DPsim::Simulation>(name, *self->pySys->sys, timestep, duration, domain, solverType, Logger::Level::INFO);

	self->channel = new EventChannel();

	return 0;
}

void DPsim::Python::Simulation::dealloc(Python::Simulation* self)
{
	if (self->thread) {
		// We have to cancel the running thread here, because otherwise self can't
		// be freed.
		Python::Simulation::stop(self, nullptr);
		self->thread->join();
		delete self->thread;
	}

	if (self->sim) {
		using SimSharedPtr = std::shared_ptr<DPsim::Simulation>;
		self->sim.~SimSharedPtr();
	}

	delete self->mut;
	delete self->cond;
	delete self->channel;

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
PyObject* DPsim::Python::Simulation::addInterface(Simulation* self, PyObject* args)
{
#ifdef WITH_SHMEM
	PyObject* pyObj;
	DPsim::Python::Interface* pyIntf;

	if (!PyArg_ParseTuple(args, "O", &pyObj))
		return nullptr;

	if (!PyObject_TypeCheck(pyObj, &DPsim::Python::InterfaceType)) {
		PyErr_SetString(PyExc_TypeError, "Argument must be dpsim.Interface");
		return nullptr;
	}

	pyIntf = (DPsim::Python::Interface*) pyObj;
	self->sim->addInterface(pyIntf->intf.get());
	Py_INCREF(pyObj);

	self->refs.push_back(pyObj);

	Py_INCREF(Py_None);
	return Py_None;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}

static const char *DocSimulationPause =
"pause()\n"
"Pause the simulation at the next possible time (usually, after finishing the current timestep).\n"
"\n"
":raises: ``SystemError`` if the simulation is not running.\n";
PyObject* DPsim::Python::Simulation::pause(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	if (self->state != State::running) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}

	newState(self, Simulation::State::pausing);
	self->cond->notify_one();

	while (self->state == State::running)
		self->cond->wait(lk);

	Py_INCREF(Py_None);
	return Py_None;
}

static const char *DocSimulationStart =
"start()\n"
"Start the simulation, or resume it if it has been paused. "
"The simulation runs in a separate thread, so this method doesn't wait for the "
"simulation to finish, but returns immediately.\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* DPsim::Python::Simulation::start(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	self->singleStepping = false;

	if (self->state == State::running) {
		PyErr_SetString(PyExc_SystemError, "Simulation already started");
		return nullptr;
	}
	else if (self->state == State::done) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	}
	else if (self->state == State::paused) {
		newState(self, Simulation::State::resuming);
		self->cond->notify_one();
	}
	else {
		newState(self, Simulation::State::starting);

		self->thread = new std::thread(DPsim::Python::Simulation::threadFunction, self);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static const char *DocSimulationStep =
"step()\n"
"Perform a single step of the simulation (possibly the first).\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* DPsim::Python::Simulation::step(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	self->singleStepping = true;

	if (self->state == State::stopped) {
		self->state = State::starting;

		self->thread = new std::thread(threadFunction, self);
	}
	else if (self->state == State::paused) {
		self->state = State::resuming;
		self->cond->notify_one();
	}
	else if (self->state == State::done) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	}
	else {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}

	while (self->state == State::running)
		self->cond->wait(lk);

	Py_INCREF(Py_None);
	return Py_None;
}

static const char *DocSimulationStop =
"stop()\n"
"Stop the simulation at the next possible time. The simulation thread is canceled "
"and the simulation can not be restarted. No-op if the simulation is not running.";
PyObject* DPsim::Python::Simulation::stop(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	if (self->state != State::running) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently not running");
		return nullptr;
	}

	newState(self, Simulation::State::stopping);

	while (self->state == State::running)
		self->cond->wait(lk);

	Py_INCREF(Py_None);
	return Py_None;
}

static const char *DocSimulationWait =
"wait()\n"
"Block until the simulation is finished, returning immediately if this is already the case.\n"
"\n"
":raises: ``SystemError`` if the simulation is paused or was not started yet.";
PyObject* DPsim::Python::Simulation::wait(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	switch (self->state) {
	case State::done:
		Py_INCREF(Py_None);
		return Py_None;

	case State::stopped:
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;

	case State::paused:
		PyErr_SetString(PyExc_SystemError, "Simulation currently paused");
		return nullptr;

	default: { }
	}

	while (self->state == State::running)
		self->cond->wait(lk);

	Py_INCREF(Py_None);
	return Py_None;
}

static const char *DocSimulationGetEventFD =
"get_eventfd(flags)\n"
"Return a poll()/select()'able file descriptor which can be used to asynchronously\n"
"notify the Python code about state changes and other events of the simulation.\n"
"\n"
":param flags: An optional mask of events which should be reported.\n"
":param coalesce: Do not report each event  but do a rate reduction instead.\n";
PyObject * DPsim::Python::Simulation::getEventFD(Simulation *self, PyObject *args) {
	int flags = -1, coalesce = 1, fd;

	if (!PyArg_ParseTuple(args, "|ii", &flags, &coalesce))
		return nullptr;

	fd = self->channel->fd();
	if (fd < 0) {
		PyErr_SetString(PyExc_SystemError, "Failed to created event file descriptor");
		return nullptr;
	}

	return Py_BuildValue("i", fd);
}

static const char *DocSimulationState =
"state\n"
"The current state of simulation.\n";
PyObject* DPsim::Python::Simulation::getState(Simulation *self, void *ctx)
{
	return Py_BuildValue("i", (int) self->state.load());
}

static const char *DocSimulationName =
"name\n"
"The name of the simulation.";
PyObject* DPsim::Python::Simulation::name(Simulation *self, void *ctx)
{
	return PyUnicode_FromString(self->sim->name().c_str());
}

PyObject* DPsim::Python::Simulation::steps(Simulation *self, void *ctx)
{
	return Py_BuildValue("i", self->sim->timeStepCount());
}

PyObject* DPsim::Python::Simulation::time(Simulation *self, void *ctx)
{
	return Py_BuildValue("f", self->sim->time());
}

PyObject* DPsim::Python::Simulation::finalTime(Simulation *self, void *ctx)
{
	return Py_BuildValue("f", self->sim->finalTime());
}

static PyGetSetDef Simulation_attrs[] = {
	{(char *) "state",      (getter) DPsim::Python::Simulation::getState, nullptr, (char *) DocSimulationState, nullptr},
	{(char *) "name",       (getter) DPsim::Python::Simulation::name,  nullptr, (char *) DocSimulationName, nullptr},
	{(char *) "steps",      (getter) DPsim::Python::Simulation::steps, nullptr, nullptr, nullptr},
	{(char *) "time",       (getter) DPsim::Python::Simulation::time,  nullptr, nullptr, nullptr},
	{(char *) "final_time", (getter) DPsim::Python::Simulation::finalTime, nullptr, nullptr, nullptr},
	{nullptr, nullptr, nullptr, nullptr, nullptr}
};

static PyMethodDef Simulation_methods[] = {
	{"add_interface", (PyCFunction) DPsim::Python::Simulation::addInterface, METH_VARARGS, (char *) DocSimulationAddInterface},
	{"pause",         (PyCFunction) DPsim::Python::Simulation::pause, METH_NOARGS, (char *) DocSimulationPause},
	{"start",         (PyCFunction) DPsim::Python::Simulation::start, METH_NOARGS, (char *) DocSimulationStart},
	{"step",          (PyCFunction) DPsim::Python::Simulation::step, METH_NOARGS,  (char *) DocSimulationStep},
	{"stop",          (PyCFunction) DPsim::Python::Simulation::stop, METH_NOARGS,  (char *) DocSimulationStop},
	{"wait",          (PyCFunction) DPsim::Python::Simulation::wait, METH_NOARGS,  (char *) DocSimulationWait},
	{"get_eventfd",   (PyCFunction) DPsim::Python::Simulation::getEventFD, METH_VARARGS, (char *) DocSimulationGetEventFD},
	{nullptr, nullptr, 0, nullptr}
};

static const char *DocSimulation =
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
	PyVarObject_HEAD_INIT(nullptr, 0)
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
	(char *) DocSimulation,                  /* tp_doc */
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
	(initproc) DPsim::Python::Simulation::init, /* tp_init */
	0,                                       /* tp_alloc */
	DPsim::Python::Simulation::newfunc,      /* tp_new */
};
