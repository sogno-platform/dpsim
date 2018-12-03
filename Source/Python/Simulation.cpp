/** Python simulation
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <chrono>
#include <cfloat>
#include <iostream>

#include <dpsim/Config.h>

#include <dpsim/Python/Simulation.h>
#include <dpsim/Python/Logger.h>
#include <dpsim/Python/Component.h>
#include <dpsim/Python/Interface.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim/SequentialScheduler.h>
#include <dpsim/ThreadLevelScheduler.h>
#include <cps/DP/DP_Ph1_Switch.h>

#ifdef WITH_OPENMP
  #include <dpsim/OpenMPLevelScheduler.h>
#endif

#ifdef WITH_SHMEM
  #include <dpsim/PthreadPoolScheduler.h>
#endif
using namespace DPsim;
using namespace CPS;

void Python::Simulation::newState(Python::Simulation *self, Simulation::State newState)
{
	uint32_t evt = static_cast<uint32_t>(newState);
	self->channel->sendEvent(evt);
	self->state = newState;
}

void Python::Simulation::threadFunction(Python::Simulation *self)
{
	Real time, finalTime;

	self->sim->schedule();

	Timer timer(Timer::Flags::fail_on_overrun);

#ifdef WITH_SHMEM
	for (auto ifm : self->sim->interfaces())
		ifm.interface->open();
#endif

	// optional start synchronization
	if (self->startSync) {
		self->sim->sync();
	}

	if (self->realTime) {
		timer.setStartTime(self->startTime);
		timer.setInterval(self->realTimeStep);
		timer.start();

		std::cout << "Starting simulation at " << self->startTime << " (delta_T = " << self->startTime - Timer::StartClock::now() << " seconds)" << std::endl;
	}

	finalTime = self->sim->finalTime();

	time = 0;
	while (time < finalTime) {
		time = self->sim->step();

		if (self->realTime) {
			try {
				timer.sleep();
			}
			catch (Timer::OverrunException) {
				std::unique_lock<std::mutex> lk(*self->mut);

				if (self->failOnOverrun) {
					newState(self, Simulation::State::overrun);
					self->cond->notify_one();
				}
			}
		}

		if (self->sim->timeStepCount() == 1) {
			std::unique_lock<std::mutex> lk(*self->mut);
			newState(self, Simulation::State::running);
			self->cond->notify_one();
		}

		if (self->state == State::pausing || self->singleStepping) {
			std::unique_lock<std::mutex> lk(*self->mut);

			newState(self, Simulation::State::paused);
			self->cond->notify_one();

			while (self->state == State::paused)
				self->cond->wait(lk);

			if (self->state == State::resuming) {
				newState(self, Simulation::State::running);
				self->cond->notify_one();
			}
		}

		if (self->state == State::stopping) {
			std::unique_lock<std::mutex> lk(*self->mut);
			newState(self, State::stopped);
			self->cond->notify_one();
			return;
		}
	}

	{
		std::unique_lock<std::mutex> lk(*self->mut);
		newState(self, State::done);
		self->cond->notify_one();
	}

	self->sim->scheduler()->stop();

#ifdef WITH_SHMEM
	for (auto ifm : self->sim->interfaces())
		ifm.interface->close();
#endif

	for (auto lg : self->sim->loggers())
		lg->close();
}

PyObject* Python::Simulation::newfunc(PyTypeObject* subtype, PyObject *args, PyObject *kwds)
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
	}

	return (PyObject*) self;
}

int Python::Simulation::init(Simulation* self, PyObject *args, PyObject *kwds)
{
	static const char *kwlist[] = {"name", "system", "timestep", "duration", "start_time", "start_time_us", "sim_type", "solver_type", "single_stepping", "rt", "rt_factor", "start_sync", "init_steady_state", "log_level", "fail_on_overrun", "split_subnets", nullptr};
	double timestep = 1e-3, duration = DBL_MAX, rtFactor = 1;
	const char *name = nullptr;
	int t = 0, s = 0, rt = 0, ss = 0, st = 0, initSteadyState = 0, splitSubnets = 1;
	int failOnOverrun = 0;

	CPS::Logger::Level logLevel = CPS::Logger::Level::INFO;

	unsigned long startTime = -1;
	unsigned long startTimeUs = 0;

	enum Solver::Type solverType;
	enum Domain domain;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|ddkkiippdppipp", (char **) kwlist,
		&name, &self->pySys, &timestep, &duration, &startTime, &startTimeUs, &s, &t, &ss, &rt, &rtFactor, &st, &initSteadyState, &logLevel, &failOnOverrun, &splitSubnets)) {
		return -1;
	}

	self->state = State::stopped;
	self->realTime = rt;
	self->startSync = ss;
	self->singleStepping = st;
	self->failOnOverrun = failOnOverrun;
	self->realTimeStep = timestep / rtFactor;

	if (startTime > 0) {
		self->startTime = Timer::StartClock::from_time_t(startTime) + std::chrono::microseconds(startTimeUs);
	}
	else
		self->startTime = Timer::StartClock::now();
	switch (s) {
		case 0: domain = Domain::DP; break;
		case 1: domain = Domain::EMT; break;
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

	if (!PyObject_TypeCheck(self->pySys, &Python::SystemTopology::type)) {
		PyErr_SetString(PyExc_TypeError, "Argument system must be dpsim.SystemTopology");
		return -1;
	}

	Py_INCREF(self->pySys);

	if (self->realTime) {
		self->sim = std::make_shared<DPsim::RealTimeSimulation>(name, *self->pySys->sys, timestep, duration, domain, solverType, logLevel, initSteadyState);
	}
	else {
		self->sim = std::make_shared<DPsim::Simulation>(name, *self->pySys->sys, timestep, duration, domain, solverType, logLevel, initSteadyState, splitSubnets);
	}
	self->channel = new EventChannel();

	return 0;
}

void Python::Simulation::dealloc(Python::Simulation* self)
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

const char* Python::Simulation::docAddEvent =
"add_switch_event(sw, time, state)\n"
"Add a switch event to the simulation.\n"
"\n"
":param sw: The Switch `Component` which should perform the switch action.\n"
":param time: The time at which the switch action should occur.\n"
":param state: Wether to open or close the switch.";
PyObject* Python::Simulation::addEvent(Simulation* self, PyObject* args)
{
	double eventTime;
	PyObject *pyObj, *pyVal;
	Python::Component *pyComp;
	const char *name;

	if (!PyArg_ParseTuple(args, "dOsO", &eventTime, &pyObj, &name, &pyVal))
		return nullptr;

	if (!PyObject_TypeCheck(pyObj, &Python::Component::type)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be of type dpsim.Component");
		return nullptr;
	}

	pyComp = (Python::Component *) pyObj;

	try {
		auto attr = pyComp->comp->attribute(name);
	}
	catch (InvalidAttributeException &) {
		PyErr_SetString(PyExc_TypeError, "Invalid attribute");
		return nullptr;
	}

	if (PyBool_Check(pyVal)) {
		Bool val = PyObject_IsTrue(pyVal);

		auto attr = pyComp->comp->attribute<Bool>(name);
		if (!attr)
			goto fail;

		auto evt = AttributeEvent<Bool>::make(eventTime, attr, val);
		self->sim->addEvent(evt);
	}
	else if (PyLong_Check(pyVal)) {
		Int val = PyLong_AsLong(pyVal);

		auto intAttr = pyComp->comp->attribute<Int>(name);
		auto uintAttr = pyComp->comp->attribute<UInt>(name);
		if (!intAttr && !uintAttr)
			goto fail;

		if (intAttr) {
			auto evt = AttributeEvent<Int>::make(eventTime, intAttr, val);
			self->sim->addEvent(evt);
		}

		if (uintAttr) {
			auto evt = AttributeEvent<UInt>::make(eventTime, uintAttr, val);
			self->sim->addEvent(evt);
		}
	}
	else if (PyFloat_Check(pyVal)) {
		double val = PyFloat_AsDouble(pyVal);

		auto attr = pyComp->comp->attribute<Real>(name);
		if (!attr)
			goto fail;

		auto evt = AttributeEvent<Real>::make(eventTime, attr, val);
		self->sim->addEvent(evt);
	}
	else if (PyComplex_Check(pyVal)) {
		Complex val(
			PyComplex_RealAsDouble(pyVal),
			PyComplex_ImagAsDouble(pyVal)
		);

		auto attr = pyComp->comp->attribute<Complex>(name);
		if (!attr)
			goto fail;

		auto evt = AttributeEvent<Complex>::make(eventTime, attr, val);
		self->sim->addEvent(evt);
	}

	Py_RETURN_NONE;

fail:
	PyErr_SetString(PyExc_TypeError, "Invalid attribute or type");
	return nullptr;
}

const char* Python::Simulation::docAddInterface =
"add_interface(intf,sync_start=True)\n"
"Add an external interface to the simulation. "
"Before each timestep, values are read from this interface and results are written to this interface afterwards. "
"See the documentation of `Interface` for more details.\n"
"\n"
":param intf: The `Interface` to be added.\n"
":param sync_start: Whether to use the interface to synchronize at the start of the simulation.";
PyObject* Python::Simulation::addInterface(Simulation* self, PyObject* args, PyObject *kwargs)
{
#ifdef WITH_SHMEM
	PyObject* pyObj;
	Python::Interface* pyIntf;
	int start_sync = 1;

	const char *kwlist[] = {"intf", "sync_start", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O|p", (char **) kwlist, &pyObj, &start_sync))
		return nullptr;

	if (!PyObject_TypeCheck(pyObj, &Python::Interface::type)) {
		PyErr_SetString(PyExc_TypeError, "Argument must be dpsim.Interface");
		return nullptr;
	}

	pyIntf = (Python::Interface*) pyObj;
	self->sim->addInterface(pyIntf->intf.get(), start_sync);
	Py_INCREF(pyObj);

	self->refs.push_back(pyObj);

	Py_RETURN_NONE;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}

const char *Python::Simulation::docAddLogger =
"add_logger(logger)";
PyObject* Python::Simulation::addLogger(Simulation *self, PyObject *args, PyObject *kwargs)
{
	PyObject *pyObj;

	const char *kwlist[] = {"logger", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O", (char **) kwlist, &pyObj))
		return nullptr;

	if (!PyObject_TypeCheck(pyObj, &Python::Logger::type)) {
		PyErr_SetString(PyExc_TypeError, "First argument must be a of type dpsim.Logger");
		return nullptr;
	}

	Python::Logger *pyLogger = (Python::Logger *) pyObj;

	self->sim->addLogger(pyLogger->logger);

	Py_RETURN_NONE;
}

const char *Python::Simulation::docPause =
"pause()\n"
"Pause the simulation at the next possible time (usually, after finishing the current timestep).\n"
"\n"
":raises: ``SystemError`` if the simulation is not running.\n";
PyObject* Python::Simulation::pause(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	if (self->state != State::running) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}

	newState(self, Simulation::State::pausing);
	self->cond->notify_one();

	while (self->state != State::paused)
		self->cond->wait(lk);

	Py_RETURN_NONE;
}

const char *Python::Simulation::docStart =
"start()\n"
"Start the simulation, or resume it if it has been paused. "
"The simulation runs in a separate thread, so this method doesn't wait for the "
"simulation to finish, but returns immediately.\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* Python::Simulation::start(Simulation *self, PyObject *args)
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
		self->cond->notify_one();

		self->thread = new std::thread(Python::Simulation::threadFunction, self);
	}

	while (self->state != State::running)
		self->cond->wait(lk);

	Py_RETURN_NONE;
}

const char *Python::Simulation::docStep =
"step()\n"
"Perform a single step of the simulation (possibly the first).\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* Python::Simulation::step(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	self->singleStepping = true;

	if (self->state == State::stopped) {
		newState(self, State::starting);
		self->cond->notify_one();

		self->thread = new std::thread(threadFunction, self);
	}
	else if (self->state == State::paused) {
		newState(self, State::resuming);
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

	while (self->state == State::starting || self->state == State::resuming || self->state == State::running)
		self->cond->wait(lk);

	Py_RETURN_NONE;
}

const char *Python::Simulation::docStop =
"stop()\n"
"Stop the simulation at the next possible time. The simulation thread is canceled "
"and the simulation can not be restarted. No-op if the simulation is not running.";
PyObject* Python::Simulation::stop(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	if (self->state != State::running && self->state != State::paused) {
		Py_RETURN_NONE;
	}

	newState(self, Simulation::State::stopping);
	self->cond->notify_one();

	while (self->state != Simulation::State::stopped)
		self->cond->wait(lk);

	Py_RETURN_NONE;
}

const char *Python::Simulation::docAddEventFD =
"add_eventfd(flags)\n";
PyObject * Python::Simulation::addEventFD(Simulation *self, PyObject *args) {
	int flags = -1, coalesce = 1, fd;

	if (!PyArg_ParseTuple(args, "i|ii", &fd, &flags, &coalesce))
		return nullptr;

	self->channel->addFd(fd);

	Py_RETURN_NONE;
}

const char *Python::Simulation::docRemoveEventFD =
"remove_eventfd(fd)\n";
PyObject * Python::Simulation::removeEventFD(Simulation *self, PyObject *args) {
	int flags = -1, coalesce = 1, fd;

	if (!PyArg_ParseTuple(args, "i", &fd, &flags, &coalesce))
		return nullptr;

	self->channel->removeFd(fd);

	Py_RETURN_NONE;
}

const char *Python::Simulation::docSetScheduler =
"set_scheduler(scheduler,...)\n"
"Set the scheduler to be used for parallel simulation, as well as "
"additional scheduler-specific parameters.\n";
PyObject* Python::Simulation::setScheduler(Simulation *self, PyObject *args, PyObject *kwargs)
{
	const char *outMeasurementFile = "";
	const char *inMeasurementFile = "";
	const char *schedName = nullptr;
	int threads = -1;
	bool useConditionVariable = false;
	bool sortTaskTypes = false;

	const char *kwlist[] = {"scheduler", "threads", "out_measurement_file", "in_measurement_file", "use_condition_variable", "sort_task_types", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "s|issbb", (char **) kwlist, &schedName, &threads, &outMeasurementFile, &inMeasurementFile, &useConditionVariable, &sortTaskTypes))
		return nullptr;

	if (!strcmp(schedName, "sequential")) {
		self->sim->setScheduler(std::make_shared<SequentialScheduler>(outMeasurementFile));
	} else if (!strcmp(schedName, "omp_level")) {
#ifdef WITH_OPENMP
		self->sim->setScheduler(std::make_shared<OpenMPLevelScheduler>(threads, outMeasurementFile));
#else
		PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
		return nullptr;
#endif
	} else if (!strcmp(schedName, "pthread_pool")) {
#ifdef WITH_SHMEM
		if (threads <= 0)
			threads = 1;
		self->sim->setScheduler(std::make_shared<PthreadPoolScheduler>(threads));
#else
		PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
		return nullptr;
#endif
	} else if (!strcmp(schedName, "thread_level")) {
		// TODO sensible default (`nproc`?)
		if (threads <= 0)
			threads = 1;
		self->sim->setScheduler(std::make_shared<ThreadLevelScheduler>(threads, outMeasurementFile, inMeasurementFile, useConditionVariable, sortTaskTypes));
	} else {
		PyErr_SetString(PyExc_ValueError, "invalid scheduler");
		return nullptr;
	}

	Py_RETURN_NONE;
}

#ifdef WITH_GRAPHVIZ
const char *Python::Simulation::docReprSVG =
"_repr_svg_()\n"
"Return a SVG graph rendering of the task dependency graph\n";
PyObject* Python::Simulation::reprSVG(Simulation *self, PyObject *args)
{
	std::stringstream ss;

	self->sim->renderDependencyGraph(ss);

	return PyUnicode_FromString(ss.str().c_str());
}
#endif

const char *Python::Simulation::docState =
"state\n"
"The current state of simulation.\n";
PyObject* Python::Simulation::getState(Simulation *self, void *ctx)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	return Py_BuildValue("i", self->state.load());
}

const char *Python::Simulation::docName =
"name\n"
"The name of the simulation.";
PyObject* Python::Simulation::name(Simulation *self, void *ctx)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	return PyUnicode_FromString(self->sim->name().c_str());
}

PyObject* Python::Simulation::steps(Simulation *self, void *ctx)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	return Py_BuildValue("i", self->sim->timeStepCount());
}

PyObject* Python::Simulation::time(Simulation *self, void *ctx)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	return Py_BuildValue("f", self->sim->time());
}

PyObject* Python::Simulation::finalTime(Simulation *self, void *ctx)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	return Py_BuildValue("f", self->sim->finalTime());
}

PyObject* Python::Simulation::avgStepTime(Simulation *self, void *ctx)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	Real tot = 0;
	for (auto meas : self->sim->stepTimes()) {
		tot += meas;
	}
	Real avg = tot / self->sim->stepTimes().size();

	return Py_BuildValue("f", avg);
}

// TODO: for everything but state, we could use read-only Attributes and a getattro
// implementation that locks the mutex before access
PyGetSetDef Python::Simulation::getset[] = {
	{(char *) "state",      (getter) Python::Simulation::getState, nullptr, (char *) Python::Simulation::docState, nullptr},
	{(char *) "name",       (getter) Python::Simulation::name,  nullptr, (char *) Python::Simulation::docName, nullptr},
	{(char *) "steps",      (getter) Python::Simulation::steps, nullptr, nullptr, nullptr},
	{(char *) "time",       (getter) Python::Simulation::time,  nullptr, nullptr, nullptr},
	{(char *) "final_time", (getter) Python::Simulation::finalTime, nullptr, nullptr, nullptr},
	{(char *) "avg_step_time", (getter) Python::Simulation::avgStepTime, nullptr, nullptr, nullptr},
	{nullptr, nullptr, nullptr, nullptr, nullptr}
};

PyMethodDef Python::Simulation::methods[] = {
	{"add_interface", (PyCFunction) Python::Simulation::addInterface, METH_VARARGS | METH_KEYWORDS, (char *) Python::Simulation::docAddInterface},
	{"add_logger",    (PyCFunction) Python::Simulation::addLogger, METH_VARARGS | METH_KEYWORDS, (char *) Python::Simulation::docAddLogger},
	{"add_event",     (PyCFunction) Python::Simulation::addEvent, METH_VARARGS, (char *) docAddEvent},
	{"pause",         (PyCFunction) Python::Simulation::pause, METH_NOARGS, (char *) Python::Simulation::docPause},
	{"start",         (PyCFunction) Python::Simulation::start, METH_NOARGS, (char *) Python::Simulation::docStart},
	{"step",          (PyCFunction) Python::Simulation::step, METH_NOARGS,  (char *) Python::Simulation::docStep},
	{"stop",          (PyCFunction) Python::Simulation::stop, METH_NOARGS,  (char *) Python::Simulation::docStop},
	{"add_eventfd",   (PyCFunction) Python::Simulation::addEventFD, METH_VARARGS, (char *) Python::Simulation::docAddEventFD},
	{"remove_eventfd",(PyCFunction) Python::Simulation::removeEventFD, METH_VARARGS, (char *) Python::Simulation::docRemoveEventFD},
	{"set_scheduler", (PyCFunction) Python::Simulation::setScheduler, METH_VARARGS | METH_KEYWORDS, (char*) Python::Simulation::docSetScheduler},
#ifdef WITH_GRAPHVIZ
	{"_repr_svg_",    (PyCFunction) Python::Simulation::reprSVG, METH_NOARGS, (char*) Python::Simulation::docReprSVG},
#endif
	{nullptr, nullptr, 0, nullptr}
};

const char *Python::Simulation::doc =
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
PyTypeObject Python::Simulation::type = {
	PyVarObject_HEAD_INIT(nullptr, 0)
	"dpsim.Simulation",                      /* tp_name */
	sizeof(Python::Simulation),       /* tp_basicsize */
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
	(char *) Simulation::doc,                /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Simulation::methods,                     /* tp_methods */
	0,                                       /* tp_members */
	Simulation::getset,                      /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc) Python::Simulation::init,     /* tp_init */
	0,                                       /* tp_alloc */
	Python::Simulation::newfunc,             /* tp_new */
};
