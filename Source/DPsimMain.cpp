#include <Python.h>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "CIMReader.h"
#include "Simulation.h"
#include "ShmemInterface.h"

using namespace DPsim;

void usage() {
	std::cerr << "usage: DPsolver [OPTIONS] CIM_FILE..." << std::endl
	  << "Possible options:" << std::endl
	  << "  -d/--duration DURATION:   simulation duration in seconds (default: 0.3)" << std::endl
	  << "  -f/--frequency FREQUENCY: system frequency in Hz (default: 50)" << std::endl
	  << "  -h/--help:                show this help and exit" << std::endl
	  << "  -i/--interface OBJ_NAME:  prefix for the names of the shmem objects used for communication (default: /dpsim)" << std::endl
	  << "  -n/--node NODE_ID:        RDF id of the node where the interfacing voltage/current source should be placed" << std::endl
	  << "  -r/--realtime:            enable realtime simulation " << std::endl
	  << "  -p/--python:              provide an interactive python shell for simulation control" << std::endl
	  << "  -s/--split INDEX:         index of this instance for distributed simulation (0 or 1)" << std::endl
	  << "  -t/--timestep TIMESTEP:   simulation timestep in seconds (default: 1e-3)" << std::endl;
}

bool parseFloat(const char *s, double *d) {
	char *end;
	*d = std::strtod(s, &end);
	return (end != s && !*end);
}

bool parseInt(const char *s, int *i) {
	char *end;
	*i = strtol(s, &end, 0);
	return (end != s && !*end);
}

std::vector<BaseComponent*> components;

enum SimState {
	StateStopped = 0,
	StateRunning,
	StatePaused
};

static std::thread *simThread;

struct SimContext {
	Simulation &sim;
	Logger &log, &llog, &rlog;
	std::condition_variable cond;
	std::mutex mut;
	std::atomic_int stop, numStep;
	SimState state;

	SimContext(Simulation&, Logger&, Logger&, Logger&);
};

SimContext::SimContext(Simulation &sim, Logger &log, Logger &llog, Logger &rlog) :
	sim(sim), log(log), llog(llog), rlog(rlog), mut(), cond() {
	this->stop = 0;
	this->numStep = 0;
	this->state = StateStopped;
}

static SimContext *globalCtx;

static void simThreadFunction(SimContext* ctx) {
	bool running = true;

	std::unique_lock<std::mutex> lk(ctx->mut, std::defer_lock);
	ctx->numStep = 0;
	while (running) {
		running = ctx->sim.step(ctx->log, ctx->llog, ctx->rlog);
		ctx->numStep++;
		ctx->sim.increaseByTimeStep();
		if (ctx->stop) {
			lk.lock();
			ctx->state = StatePaused;
			ctx->cond.notify_one();
			ctx->cond.wait(lk);
			ctx->state = StateRunning;
			lk.unlock();
		}
	}
	lk.lock();
	ctx->state = StateStopped;
	ctx->cond.notify_one();
}

static PyObject* pythonStart(PyObject *self, PyObject *args) {
	std::unique_lock<std::mutex> lk(globalCtx->mut);
	if (globalCtx->state != StateStopped) {
		PyErr_SetString(PyExc_SystemError, "Simulation already started");
		return nullptr;
	}
	globalCtx->stop = 0;
	globalCtx->state = StateRunning;
	simThread = new std::thread(simThreadFunction, globalCtx);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pythonStep(PyObject *self, PyObject *args) {
	std::unique_lock<std::mutex> lk(globalCtx->mut);
	int oldStep = globalCtx->numStep;
	if (globalCtx->state == StateStopped) {
		globalCtx->state = StateRunning;
		globalCtx->stop = 1;
		simThread = new std::thread(simThreadFunction, globalCtx);
	} else if (globalCtx->state == StatePaused) {
		globalCtx->stop = 1;
		globalCtx->cond.notify_one();
	} else {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}
	while (globalCtx->numStep == oldStep)
		globalCtx->cond.wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pythonPause(PyObject *self, PyObject *args) {
	std::unique_lock<std::mutex> lk(globalCtx->mut);
	if (globalCtx->state != StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}
	globalCtx->stop = 1;
	globalCtx->cond.notify_one();
	while (globalCtx->state == StateRunning)
		globalCtx->cond.wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pythonWait(PyObject *self, PyObject *args) {
	std::unique_lock<std::mutex> lk(globalCtx->mut);
	if (globalCtx->state == StateStopped) {
		Py_INCREF(Py_None);
		return Py_None;
	} else if (globalCtx->state == StatePaused) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently paused");
		return nullptr;
	}
	while (globalCtx->state == StateRunning)
		globalCtx->cond.wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyMethodDef pythonMethods[] = {
	{"start", pythonStart, METH_VARARGS, "Start the simulation."},
	{"step", pythonStep, METH_VARARGS, "Perform a single simulation step."},
	{"pause", pythonPause, METH_VARARGS, "Pause the already running simulation."},
	{"wait", pythonWait, METH_VARARGS, "Wait for the simulation to finish."},
	{NULL, NULL, 0, NULL}
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "dpsim", NULL, -1, pythonMethods,
	NULL, NULL, NULL, NULL
};

static PyObject* PyInit_dpsim(void) {
	return PyModule_Create(&dpsimModule);
}

int doPythonLoop(SimContext* sim) {
	PyImport_AppendInittab("dpsim", &PyInit_dpsim);
	Py_Initialize();

	while (std::cin.good() && Py_IsInitialized()) {
		std::cout << "> ";
		std::string line;
		std::getline(std::cin, line);
		PyRun_SimpleString(line.c_str());
	}
}

// TODO: that many platform-dependent ifdefs inside main are kind of ugly
int cimMain(int argc, const char* argv[]) {
	bool rt = false, python = false;
	int i, split = -1;
	Real frequency = 2*PI*50, timestep = 0.001, duration = 0.3;
	std::string interfaceBase = "/dpsim";
	std::string splitNode = "";
	std::string outName, inName, logName("log.txt"), llogName("lvector.csv"), rlogName("rvector.csv");
#ifdef __linux__
	ShmemInterface *intf = nullptr;
#endif

	// Parse arguments
	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-d") || !strcmp(argv[i], "--duration")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -d/--duration; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseFloat(argv[++i], &duration) || duration <= 0) {
				std::cerr << "Invalid setting " << argv[i] << " for the duration" << std::endl;
				return 1;
			}
		} else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--frequency")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -f/--frequency; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseFloat(argv[++i], &frequency) || frequency <= 0) {
				std::cerr << "Invalid setting " << argv[i] << " for system frequency" << std::endl;
				return 1;
			}
			frequency *= 2*PI;
		} else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			usage();
			return 0;
		} else if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "--interface")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -i/--interface; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (argv[++i][0] != '/') {
				std::cerr << "Shmem interface object name must start with a '/'" << std::endl;
				return 1;
			}
			interfaceBase = std::string(argv[i]);
		} else if (!strcmp(argv[i], "-n") || !strcmp(argv[i], "--node")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -n/--node; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			splitNode = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--python")) {
			python = true;
		} else if (!strcmp(argv[i], "-r") || !strcmp(argv[i], "--realtime")) {
			rt = true;
		} else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--split")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -s/--split; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseInt(argv[++i], &split) || split < 0 || split > 1) {
				std::cerr << "Invalid setting " << argv[i] << " for the split index" << std::endl;
				return 1;
			}
		} else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--timestep")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -t/--timestep; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseFloat(argv[++i], &timestep) || timestep <= 0) {
				std::cerr << "Invalied setting " << argv[i] << " for the timestep" << std::endl;
				return 1;
			}
		} else if (argv[i][0] == '-') {
			std::cerr << "Unknown option " << argv[i] << " ; see 'DPsim --help' for usage" << std::endl;
			return 1;
		} else {
			// remaining arguments treated as input files
			break;
		}
	}
	if (i == argc) {
		std::cerr << "No input files given (see DPsim --help for usage)" << std::endl;
		return 1;
	}
#ifndef __linux__
	if (split >= 0 || splitNode.length() != 0) {
		std::cerr << "Distributed simulation not supported on this platform" << std::endl;
		return 1;
	} else if (rt) {
		std::cerr << "Realtime simulation not supported on this platform" << std::endl;
		return 1;
	}
#endif
	if (python && (split >= 0 || splitNode.length() != 0 || rt)) {
		std::cerr << "Realtime and distributed simulation currently not supported in combination with Python" << std::endl;
		return 1;
	}

	// Parse CIM files
	CIMReader reader(frequency);
	for (; i < argc; i++) {
		if (!reader.addFile(argv[i])) {
			std::cerr << "Failed to read file " << argv[i] << std::endl;
			return 1;
		}
	}
	reader.parseFiles();
	components = reader.getComponents();

#ifdef __linux__
	// TODO: this is a simple, pretty much fixed setup. Make this more flexible / configurable
	if (split >= 0) {
		int node = reader.mapTopologicalNode(splitNode);
		if (node < 0) {
			std::cerr << "Invalid / missing split node" << std::endl;
			return 1;
		}
		if (split == 0) {
			outName = interfaceBase + ".0.out";
			inName = interfaceBase + ".0.in";
			intf = new ShmemInterface(outName.c_str(), inName.c_str());
			ExternalVoltageSource *evs = new ExternalVoltageSource("v_int", node, 0, 0, 0, reader.getNumVoltageSources()+1);
			intf->registerVoltageSource(evs, 0, 1);
			intf->registerExportedCurrent(evs, 0, 1);
			components.push_back(evs);
			// TODO make log names configurable
			logName = "cim0.log";
			llogName = "lvector-cim0.csv";
			rlogName = "rvector-cim0.csv";
		} else {
			outName = interfaceBase + ".1.out";
			inName = interfaceBase + ".1.in";
			intf = new ShmemInterface(outName.c_str(), inName.c_str());
			ExternalCurrentSource *ecs = new ExternalCurrentSource("i_int", node, 0, 0, 0);
			intf->registerCurrentSource(ecs, 0, 1);
			intf->registerExportedVoltage(node, 0, 0, 1);
			components.push_back(ecs);
			logName = "cim1.log";
			llogName = "lvector-cim1.csv";
			rlogName = "rvector-cim1.csv";
		}
	}
#endif

	// Do the actual simulation
	Logger log(logName), llog(llogName), rlog(rlogName);
	Simulation sim(components, frequency, timestep, duration, log);
	
	if (python) {
		globalCtx = new SimContext(sim, log, llog, rlog);
		doPythonLoop(globalCtx);
	} else {
#ifdef __linux__
		if (intf)
			sim.addExternalInterface(intf);

		if (rt) {
			sim.runRT(RTTimerFD, true, log, llog, rlog);
		} else {
			while (sim.step(log, llog, rlog))
				sim.increaseByTimeStep();
		}

		if (intf)
			delete intf;
#else
		while (sim.step(log, llog, rlog))
			sim.increaseByTimeStep();
#endif
	}
	return 0;
}

int main(int argc, const char* argv[]) {
	return cimMain(argc, argv);
}
