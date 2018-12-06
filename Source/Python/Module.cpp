/** Python module
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

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <dpsim/Config.h>
#include <dpsim/Python/Module.h>
#include <dpsim/Python/Component.h>
#include <dpsim/Python/Node.h>
#include <dpsim/Python/SystemTopology.h>
#include <dpsim/Python/Simulation.h>
#include <dpsim/Python/LoadCim.h>
#include <dpsim/Python/Logger.h>
#ifndef _MSC_VER
#include <dpsim/Python/Interface.h>
#endif

#include <cps/Components.h>

using namespace DPsim::Python;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim",               (PyCFunction) LoadCim,       METH_VARARGS | METH_KEYWORDS, DPsim::Python::DocLoadCim },

	// Component constructors
	Component::constructorDef<CPS::DP::Ph1::Capacitor>("_dp_ph1_Capacitor"),
	Component::constructorDef<CPS::DP::Ph1::CurrentSource>("_dp_ph1_CurrentSource"),
	Component::constructorDef<CPS::DP::Ph1::Inductor>("_dp_ph1_Inductor"),
	Component::constructorDef<CPS::DP::Ph1::PQLoadCS>("_dp_ph1_PQLoadCS"),
	Component::constructorDef<CPS::DP::Ph1::PiLine>("_dp_ph1_PiLine"),
	Component::constructorDef<CPS::DP::Ph1::RXLoad>("_dp_ph1_RXLoad"),
	Component::constructorDef<CPS::DP::Ph1::Resistor>("_dp_ph1_Resistor"),
	Component::constructorDef<CPS::DP::Ph1::RxLine>("_dp_ph1_RxLine"),
	Component::constructorDef<CPS::DP::Ph1::Switch>("_dp_ph1_Switch"),
	Component::constructorDef<CPS::DP::Ph1::SynchronGeneratorIdeal>("_dp_ph1_SynchronGeneratorIdeal"),
	Component::constructorDef<CPS::DP::Ph1::SynchronGeneratorTrStab>("_dp_ph1_SynchronGeneratorTrStab"),
	Component::constructorDef<CPS::DP::Ph1::Transformer>("_dp_ph1_Transformer"),
	Component::constructorDef<CPS::DP::Ph1::VoltageSource>("_dp_ph1_VoltageSource"),
	Component::constructorDef<CPS::DP::Ph1::VoltageSourceFreq>("_dp_ph1_VoltageSourceFreq"),
	Component::constructorDef<CPS::DP::Ph1::VoltageSourceNorton>("_dp_ph1_VoltageSourceNorton"),
	Component::constructorDef<CPS::DP::Ph3::SeriesResistor>("_dp_ph3_SeriesResistor"),
	Component::constructorDef<CPS::DP::Ph3::SeriesSwitch>("_dp_ph3_SeriesSwitch"),
	Component::constructorDef<CPS::DP::Ph3::SynchronGeneratorDQ>("_dp_ph3_SynchronGeneratorDQ"),
	Component::constructorDef<CPS::EMT::Ph1::Capacitor>("_emt_ph1_Capacitor"),
	Component::constructorDef<CPS::EMT::Ph1::CurrentSource>("_emt_ph1_CurrentSource"),
	Component::constructorDef<CPS::EMT::Ph1::Inductor>("_emt_ph1_Inductor"),
	Component::constructorDef<CPS::EMT::Ph1::Resistor>("_emt_ph1_Resistor"),
	Component::constructorDef<CPS::EMT::Ph1::VoltageSource>("_emt_ph1_VoltageSource"),
	Component::constructorDef<CPS::EMT::Ph1::VoltageSourceFreq>("_emt_ph1_VoltageSourceFreq"),
	Component::constructorDef<CPS::EMT::Ph1::VoltageSourceNorton>("_emt_ph1_VoltageSourceNorton"),

	{ nullptr }
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "_dpsim", nullptr, -1, dpsimModuleMethods, nullptr, nullptr, nullptr, nullptr
};

PyMODINIT_FUNC PyInit__dpsim(void) {
	PyObject* m;

	if (PyType_Ready(&Component::type) < 0)
		return nullptr;
	if (PyType_Ready(&Node<CPS::Real>::type) < 0)
		return nullptr;
	if (PyType_Ready(&Node<CPS::Complex>::type) < 0)
		return nullptr;
	if (PyType_Ready(&Simulation::type) < 0)
		return nullptr;
	if (PyType_Ready(&SystemTopology::type) < 0)
		return nullptr;
	if (PyType_Ready(&Logger::type) < 0)
		return nullptr;
#ifdef WITH_SHMEM
	if (PyType_Ready(&Interface::type) < 0)
		return nullptr;
#endif

	m = PyModule_Create(&dpsimModule);
	if (!m)
		return nullptr;

	PyModule_AddStringConstant(m, "__version__", DPSIM_RELEASE);
	Py_INCREF(&Simulation::type);
	PyModule_AddObject(m, "Simulation", (PyObject*) &Simulation::type);
	Py_INCREF(&SystemTopology::type);
	PyModule_AddObject(m, "SystemTopology", (PyObject*) &SystemTopology::type);
	Py_INCREF(&Logger::type);
	PyModule_AddObject(m, "Logger", (PyObject*) &Logger::type);
	Py_INCREF(&Component::type);
	PyModule_AddObject(m, "Component", (PyObject*) &Component::type);
#ifdef WITH_SHMEM
	Py_INCREF(&Interface::type);
	PyModule_AddObject(m, "Interface", (PyObject*) &Interface::type);
#endif
	Py_INCREF(&Node<CPS::Complex>::type);
	PyModule_AddObject(m, "_dp_Node", (PyObject*) &Node<CPS::Complex>::type);
	Py_INCREF(&Node<CPS::Real>::type);
	PyModule_AddObject(m, "_emt_Node", (PyObject*) &Node<CPS::Real>::type);

	return m;
}
