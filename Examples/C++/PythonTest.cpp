/** Python Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

#ifdef __python__

#include "PythonTest.h"

using namespace DPsim;

static PyMethodDef pyModuleMethods[] = {
	{ "load_cim", pyLoadCim, METH_VARARGS, "Load a network from CIM file(s)." },
	{ 0 }
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "dpsim", NULL, -1, pyModuleMethods,
	NULL, NULL, NULL, NULL
};

static PyObject* PyInit_dpsim(void) {
	PyObject* m;

	if (PyType_Ready(&PyComponentType) < 0)
		return nullptr;
	if (PyType_Ready(&PySimulationType) < 0)
		return nullptr;

	m = PyModule_Create(&dpsimModule);
	if (!m)
		return nullptr;

	Py_INCREF(&PySimulationType);
	PyModule_AddObject(m, "Simulation", (PyObject*)&PySimulationType);
	Py_INCREF(&PyComponentType);
	PyModule_AddObject(m, "Component", (PyObject*)&PyComponentType);
	return m;
}


void DPsim::pythonExampleMain() {
	PyImport_AppendInittab("dpsim", &PyInit_dpsim);
	Py_Initialize();
	for (; i < argc; i++) {
		FILE* f = fopen(argv[i], "r");
		if (!f) {
			std::cerr << "Failed to open ";
			std::perror(argv[i]);
			return 1;
		}
		PyRun_SimpleFile(f, argv[i]);
		fclose(f);
	}

	if (!batch) {
		while (std::cin.good() && Py_IsInitialized()) {
			std::cout << "> ";
			std::string line;
			std::getline(std::cin, line);
			PyRun_SimpleString(line.c_str());
		}
	}
}

#endif