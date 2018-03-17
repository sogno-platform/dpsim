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

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include "Config.h"
#include "CPowerSystems/Source/Components.h"

#include "Python/Component.h"
#include "Python/LoadCim.h"
#include "Python/Interface.h"
#include "Python/Module.h"
#include "Python/Simulation.h"

#include "Python/Components/LinePi.h"
#include "Python/Components/LineRx.h"
#include "Python/Components/LoadPQ.h"
#include "Python/Components/Capacitor.h"
#include "Python/Components/Resistor.h"
#include "Python/Components/Inductor.h"
#include "Python/Components/CurrentSource.h"
#include "Python/Components/VoltageSource.h"
#include "Python/Components/VoltageSourceNorton.h"

using namespace DPsim;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim", Python::LoadCim, METH_VARARGS, Python::DocLoadCim },
	{ "open_shmem_interface", (PyCFunction) Python::OpenShmemInterface, METH_VARARGS|METH_KEYWORDS, Python::DocOpenShmemInterface },

	// Component constructors
	{ "CurrentSourceDP",        Python::Components::CurrentSource<DPsim::Components::DP::CurrentSource>,              METH_VARARGS, Python::Components::DocCurrentSource },
	{ "CurrentSourceEMT",       Python::Components::CurrentSource<DPsim::Components::EMT::CurrentSource>,             METH_VARARGS, Python::Components::DocCurrentSource },
	{ "VoltageSourceDP",        Python::Components::VoltageSource<DPsim::Components::DP::VoltageSource>,              METH_VARARGS, Python::Components::DocVoltageSource },
	{ "VoltageSourceEMT",       Python::Components::VoltageSource<DPsim::Components::EMT::VoltageSource>,             METH_VARARGS, Python::Components::DocVoltageSource },
	{ "VoltageSourceNortonDP",  Python::Components::VoltageSourceNorton<DPsim::Components::DP::VoltageSourceNorton>,  METH_VARARGS, Python::Components::DocVoltageSourceNorton },
	{ "VoltageSourceNortonEMT", Python::Components::VoltageSourceNorton<DPsim::Components::EMT::VoltageSourceNorton>, METH_VARARGS, Python::Components::DocVoltageSourceNorton },
	{ "InductorDP",             Python::Components::Inductor<DPsim::Components::DP::Inductor>,                        METH_VARARGS, Python::Components::DocInductor },
	{ "InductorEMT",            Python::Components::Inductor<DPsim::Components::EMT::Inductor>,                       METH_VARARGS, Python::Components::DocInductor },
	{ "ResistorDP",	            Python::Components::Resistor<DPsim::Components::DP::Resistor>,                        METH_VARARGS, Python::Components::DocResistor },
	{ "ResistorEMT",            Python::Components::Resistor<DPsim::Components::EMT::Resistor>,                       METH_VARARGS, Python::Components::DocResistor },
	{ "CapacitorDP",            Python::Components::Capacitor<DPsim::Components::DP::Capacitor>,                      METH_VARARGS, Python::Components::DocCapacitor },
	{ "CapacitorEMT",           Python::Components::Capacitor<DPsim::Components::EMT::Capacitor>,                     METH_VARARGS, Python::Components::DocCapacitor },
	{ "LinePiDP",               Python::Components::LinePi<DPsim::Components::DP::PiLine>,                            METH_VARARGS, Python::Components::DocLinePi },
	{ "LineRxDP",               Python::Components::LineRx<DPsim::Components::DP::RxLine>,                            METH_VARARGS, Python::Components::DocLineRx },
	{ "LoadPQDP",               Python::Components::LoadPQ<DPsim::Components::DP::PQLoad>,                            METH_VARARGS, Python::Components::DocLoadPQ },
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
