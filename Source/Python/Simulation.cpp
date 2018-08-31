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
#include <dpsim/Python/Simulation.h>

#include <dpsim/Python/Interface.h>
#include <dpsim/Python/Component.h>

#include <cfloat>
#include <iostream>

using namespace DPsim;
using namespace CPS;

void DPsim::Python::Simulation::simThreadFunction(DPsim::Python::Simulation *self)
{
	Real time, endTime;
	std::unique_lock<std::mutex> lk(*self->mut, std::defer_lock);

	endTime = self->sim->finalTime();
#ifdef __linux__
	self->sim->sendEvent(DPsim::Simulation::Event::Started);
#endif
	self->numStep = 0;
	while (self->running && time < endTime) {
		time = self->sim->step();

		self->numStep++;

		if (self->sigPause) {
			lk.lock();
			self->simState = State::Paused;
#ifdef __linux__
			self->sim->sendEvent(DPsim::Simulation::Event::Paused);
#endif
			self->cond->notify_one();
			self->cond->wait(lk);

			self->simState = State::Running;
#ifdef __linux__
			self->sim->sendEvent(DPsim::Simulation::Event::Resumed);
#endif
			lk.unlock();
		}
	}
	lk.lock();
#ifdef __linux__
	self->sim->sendEvent(DPsim::Simulation::Event::Finished);
#endif
	self->simState = State::Done;
	self->cond->notify_one();
}

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

int DPsim::Python::Simulation::init(Simulation* self, PyObject *args, PyObject *kwds)
{
	static char *kwlist[] = {"name", "system", "timestep", "duration", "sim_type", "solver_type", nullptr};
	double timestep = 1e-3, duration = DBL_MAX;
	const char *name = nullptr;
	int t = 0, s = 0;

	enum Solver::Type solverType;
	enum Domain domain;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|ddii", kwlist,
		&name, &self->pySys, &timestep, &duration, &s, &t)) {
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

	if (!PyObject_TypeCheck(self->pySys, &DPsim::Python::SystemTopologyType)) {
		PyErr_SetString(PyExc_TypeError, "Argument system must be dpsim.SystemTopology");
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
		Python::Simulation::stop(self, nullptr);
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

#if 0
static const char* DocSimulationLvector =
"lvector()\n"
"Return the left-side vector of the last step as a list of floats.";
PyObject*DPsim::Python::Simulation::lvector(PyObject *self, PyObject *args)
{
	if (self->simState == State::Running) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}

	Matrix& lvector = self->sim->leftSideVector();
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
PyObject* DPsim::Python::Simulation::pause(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	if (self->simState != State::Running) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}

	self->sigPause = 1;
	self->cond->notify_one();

	while (self->simState == State::Running)
		self->cond->wait(lk);

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
PyObject* DPsim::Python::Simulation::start(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	if (self->simState == State::Running) {
		PyErr_SetString(PyExc_SystemError, "Simulation already started");
		return nullptr;
	}
	else if (self->simState == State::Done) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	}
	else if (self->simState == State::Paused) {
		self->sigPause = 0;
		self->cond->notify_one();
	}
	else {
		self->sigPause = 0;
		self->simState = State::Running;
		self->running = true;
		self->simThread = new std::thread(simThreadFunction, self);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static const char* DocSimulationStep =
"step()\n"
"Perform a single step of the simulation (possibly the first).\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* DPsim::Python::Simulation::step(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	int oldStep = self->numStep;
	if (self->simState == State::Stopped) {
		self->simState = State::Running;
		self->sigPause = 1;
		self->running = true;
		self->simThread = new std::thread(simThreadFunction, self);
	}
	else if (self->simState == State::Paused) {
		self->sigPause = 1;
		self->cond->notify_one();
	}
	else if (self->simState == State::Done) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	}
	else {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}

	while (self->numStep == oldStep)
		self->cond->wait(lk);

	Py_INCREF(Py_None);
	return Py_None;
}

static const char* DocSimulationStop =
"stop()\n"
"Stop the simulation at the next possible time. The simulation thread is canceled "
"and the simulation can not be restarted. No-op if the simulation is not running.";
PyObject* DPsim::Python::Simulation::stop(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);
	self->running = false;

	while (self->simState == State::Running)
		self->cond->wait(lk);

	Py_INCREF(Py_None);
	return Py_None;
}

static const char* DocSimulationWait =
"wait()\n"
"Block until the simulation is finished, returning immediately if this is already the case.\n"
"\n"
":raises: ``SystemError`` if the simulation is paused or was not started yet.";
PyObject* DPsim::Python::Simulation::wait(Simulation *self, PyObject *args)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	if (self->simState == State::Done) {
		Py_INCREF(Py_None);
		return Py_None;
	}
	else if (self->simState == State::Stopped) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}
	else if (self->simState == State::Paused) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently paused");
		return nullptr;
	}

	while (self->simState == State::Running)
		self->cond->wait(lk);

	Py_INCREF(Py_None);
	return Py_None;
}

#ifdef __linux__
static char* DocSimulationEventFD =
"get_eventfd(flags)\n"
"Return a poll()/select()'able file descriptor which can be used to asynchronously\n"
"notify the Python code about state changes and other events of the simulation.\n"
"\n"
":param flags: An optional mask of events which should be reported.\n"
":param coalesce: Do not report each event  but do a rate reduction instead.\n";
PyObject * DPsim::Python::Simulation::eventFD(Simulation *self, PyObject *args) {
	int flags = -1, coalesce = 1, fd;

	if (!PyArg_ParseTuple(args, "|ii", &flags, &coalesce))
		return nullptr;

	fd = self->sim->eventFD(flags, coalesce);
	if (fd < 0) {
		PyErr_SetString(PyExc_SystemError, "Failed to created event file descriptor");
		return nullptr;
	}

	return Py_BuildValue("i", fd);
}
#endif

static char* DocSimulationState =
"state\n"
"The current state of simulation.\n";
PyObject* DPsim::Python::Simulation::state(Simulation *self, void *ctx)
{
	std::unique_lock<std::mutex> lk(*self->mut);

	return Py_BuildValue("i", self->simState);
}

static char* DocSimulationName =
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
	{"state",      (getter) DPsim::Python::Simulation::state, nullptr, DocSimulationState, nullptr},
	{"name",       (getter) DPsim::Python::Simulation::name, nullptr, DocSimulationName, nullptr},
	{"steps",      (getter) DPsim::Python::Simulation::steps, nullptr, nullptr, nullptr},
	{"time",       (getter) DPsim::Python::Simulation::time, nullptr, nullptr, nullptr},
	{"final_time", (getter) DPsim::Python::Simulation::finalTime, nullptr, nullptr, nullptr},
	{nullptr, nullptr, nullptr, nullptr, nullptr}
};

static PyMethodDef Simulation_methods[] = {
	{"add_interface", (PyCFunction) DPsim::Python::Simulation::addInterface, METH_VARARGS, DocSimulationAddInterface},
	{"pause",         (PyCFunction) DPsim::Python::Simulation::pause, METH_NOARGS, DocSimulationPause},
	{"start",         (PyCFunction) DPsim::Python::Simulation::start, METH_NOARGS, DocSimulationStart},
	{"step",          (PyCFunction) DPsim::Python::Simulation::step, METH_NOARGS, DocSimulationStep},
	{"stop",          (PyCFunction) DPsim::Python::Simulation::stop, METH_NOARGS, DocSimulationStop},
	{"wait",          (PyCFunction) DPsim::Python::Simulation::wait, METH_NOARGS, DocSimulationWait},
#ifdef __linux__
	{"eventfd",       (PyCFunction) DPsim::Python::Simulation::eventFD, METH_VARARGS, DocSimulationEventFD},
#endif
	{nullptr, nullptr, 0, nullptr}
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
