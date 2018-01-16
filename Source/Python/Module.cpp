/** Python module
 *
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

#include <Python.h>

#include "Config.h"
#include "Python/Component.h"
#include "Python/LoadCim.h"
#include "Python/Interface.h"
#include "Python/Module.h"
#include "Python/Simulation.h"

using namespace DPsim;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim", Python::LoadCim, METH_VARARGS, Python::DocLoadCim },
	{ "open_shmem_interface", (PyCFunction) Python::OpenShmemInterface, METH_VARARGS|METH_KEYWORDS, Python::DocOpenShmemInterface },
	{ "CurrentSourceDP", Python::Components::DP::CurrentSource, METH_VARARGS, Python::Components::DocCurrentSource },
	{ "VoltageSourceDP", Python::Components::DP::VoltageSource, METH_VARARGS, Python::Components::DocVoltageSource },
	{ "VoltageSourceNortonDP", Python::Components::DP::VoltageSourceNorton, METH_VARARGS, Python::Components::DocVoltageSourceNorton },
	{ "ResistorDP",  Python::Components::DP::Resistor,  METH_VARARGS, Python::Components::DocResistor },
	{ "CapacitorDP", Python::Components::DP::Capacitor, METH_VARARGS, Python::Components::DocCapacitor },
	{ "InductorDP",  Python::Components::DP::Inductor,  METH_VARARGS, Python::Components::DocInductor },
	{ "InductorEMT", Python::Components::EMT::Inductor, METH_VARARGS, Python::Components::DocInductor },
	{ "ResistorEMT", Python::Components::EMT::Resistor, METH_VARARGS, Python::Components::DocResistor },
	{ 0 }
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "_dpsim", NULL, -1, dpsimModuleMethods, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit__dpsim(void) {
	PyObject* m;

	if (PyType_Ready(&Python::ComponentType) < 0)
		return nullptr;
	if (PyType_Ready(&Python::SimulationType) < 0)
		return nullptr;
	Python::InterfaceType.tp_new = PyType_GenericNew;
	if (PyType_Ready(&Python::InterfaceType) < 0)
		return nullptr;

	m = PyModule_Create(&dpsimModule);
	if (!m)
		return nullptr;

	Py_INCREF(&Python::SimulationType);
	PyModule_AddObject(m, "Simulation", (PyObject*) &Python::SimulationType);
	Py_INCREF(&Python::ComponentType);
	PyModule_AddObject(m, "Component", (PyObject*) &Python::ComponentType);
	Py_INCREF(&Python::InterfaceType);
	PyModule_AddObject(m, "Interface", (PyObject*) &Python::InterfaceType);

	return m;
}
