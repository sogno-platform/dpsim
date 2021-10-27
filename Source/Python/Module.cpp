/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
#endif

#include <cps/Components.h>

using namespace DPsim::Python;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim",               (PyCFunction) LoadCim,       METH_VARARGS | METH_KEYWORDS, DPsim::Python::DocLoadCim },

	// Dynamic Phasor (DP)
	Component::constructorDef<CPS::DP::Ph1::Capacitor>("_dp_ph1_Capacitor"),
	Component::constructorDef<CPS::DP::Ph1::CurrentSource>("_dp_ph1_CurrentSource"),
	Component::constructorDef<CPS::DP::Ph1::Inductor>("_dp_ph1_Inductor"),
	Component::constructorDef<CPS::DP::Ph1::Inverter>("_dp_ph1_Inverter"),
	Component::constructorDef<CPS::DP::Ph1::PiLine>("_dp_ph1_PiLine"),
	Component::constructorDef<CPS::DP::Ph1::PQLoadCS>("_dp_ph1_PQLoadCS"),
	Component::constructorDef<CPS::DP::Ph1::Resistor>("_dp_ph1_Resistor"),
	Component::constructorDef<CPS::DP::Ph1::RxLine>("_dp_ph1_RxLine"),
	Component::constructorDef<CPS::DP::Ph1::RXLoad>("_dp_ph1_RXLoad"),
	Component::constructorDef<CPS::DP::Ph1::Switch>("_dp_ph1_Switch"),
	Component::constructorDef<CPS::DP::Ph1::SynchronGeneratorIdeal>("_dp_ph1_SynchronGeneratorIdeal"),
	Component::constructorDef<CPS::DP::Ph1::SynchronGeneratorTrStab>("_dp_ph1_SynchronGeneratorTrStab"),
	Component::constructorDef<CPS::DP::Ph1::Transformer>("_dp_ph1_Transformer"),
	Component::constructorDef<CPS::DP::Ph1::VoltageSource>("_dp_ph1_VoltageSource"),
	Component::constructorDef<CPS::DP::Ph1::VoltageSourceNorton>("_dp_ph1_VoltageSourceNorton"),
	Component::constructorDef<CPS::DP::Ph1::VoltageSourceNorton>("_dp_ph1_VoltageSourceNorton"),
	Component::constructorDef<CPS::DP::Ph1::VoltageSourceRamp>("_dp_ph1_VoltageSourceRamp"),

	Component::constructorDef<CPS::DP::Ph3::SeriesResistor>("_dp_ph3_SeriesResistor"),
	Component::constructorDef<CPS::DP::Ph3::SeriesSwitch>("_dp_ph3_SeriesSwitch"),
	// Component::constructorDef<CPS::DP::Ph3::SynchronGeneratorDQSmpl>("_dp_ph3_SynchronGeneratorDQSmpl"),
	Component::constructorDef<CPS::DP::Ph3::SynchronGeneratorDQTrapez>("_dp_ph3_SynchronGeneratorDQTrapez"),
	// Component::constructorDef<CPS::DP::Ph3::SynchronGeneratorVBR>("_dp_ph3_SynchronGeneratorVBR"),
	// Component::constructorDef<CPS::DP::Ph3::SynchronGeneratorVBRStandalone>("_dp_ph3_SynchronGeneratorVBRStandalone"),

#ifdef WITH_SUNDIALS
	Component::constructorDef<CPS::DP::Ph3::SynchronGeneratorDQODE>("_dp_ph3_SynchronGeneratorDQODE"),
#endif

	// Electro Magnetic Transients (EMT)
	Component::constructorDef<CPS::EMT::Ph1::Capacitor>("_emt_ph1_Capacitor"),
	Component::constructorDef<CPS::EMT::Ph1::CurrentSource>("_emt_ph1_CurrentSource"),
	Component::constructorDef<CPS::EMT::Ph1::Inductor>("_emt_ph1_Inductor"),
	Component::constructorDef<CPS::EMT::Ph1::Resistor>("_emt_ph1_Resistor"),
	Component::constructorDef<CPS::EMT::Ph1::VoltageSource>("_emt_ph1_VoltageSource"),
	Component::constructorDef<CPS::EMT::Ph1::VoltageSourceNorton>("_emt_ph1_VoltageSourceNorton"),
	Component::constructorDef<CPS::EMT::Ph1::VoltageSourceRamp>("_emt_ph1_VoltageSourceRamp"),

	// Component::constructorDef<CPS::EMT::Ph3::SynchronGeneratorDQ>("_emt_ph3_SynchronGeneratorDQ"),
	// Component::constructorDef<CPS::EMT::Ph3::SynchronGeneratorDQSmpl>("_emt_ph3_SynchronGeneratorDQSmpl"),
	// Component::constructorDef<CPS::EMT::Ph3::SynchronGeneratorDQSmplCompSource>("_emt_ph3_SynchronGeneratorDQSmplCompSource"),
	// Component::constructorDef<CPS::EMT::Ph3::SynchronGeneratorVBR>("_emt_ph3_SynchronGeneratorVBR"),
	// Component::constructorDef<CPS::EMT::Ph3::SynchronGeneratorVBRSmpl>("_emt_ph3_SynchronGeneratorVBRSmpl"),
	// Component::constructorDef<CPS::EMT::Ph3::SynchronGeneratorVBRStandalone>("_emt_ph3_SynchronGeneratorVBRStandalone"),

	// Static Phasor (SP)
	Component::constructorDef<CPS::SP::Ph1::NetworkInjection>("_sp_ph1_externalGridInjection"),
	Component::constructorDef<CPS::SP::Ph1::Load>("_sp_ph1_Load"),
	Component::constructorDef<CPS::SP::Ph1::PiLine>("_sp_ph1_PiLine"),
	Component::constructorDef<CPS::SP::Ph1::PQNode>("_sp_ph1_PQNode"),
	Component::constructorDef<CPS::SP::Ph1::PVNode>("_sp_ph1_PVNode"),
	Component::constructorDef<CPS::SP::Ph1::Shunt>("_sp_ph1_Shunt"),
	Component::constructorDef<CPS::SP::Ph1::SynchronGenerator>("_sp_ph1_SynchronGenerator"),
	Component::constructorDef<CPS::SP::Ph1::Transformer>("_sp_ph1_Transformer"),
	Component::constructorDef<CPS::SP::Ph1::VDNode>("_sp_ph1_VDNode"),

	// Control Signal
	// ToDo: Support signal components in Python
	// Component::constructorDef<CPS::Signal::DecouplingLine>("_signal_DecouplingLine"),
	// Component::constructorDef<CPS::Signal::Exciter>("_signal_Exciter"),
	// Component::constructorDef<CPS::Signal::FIRFilter>("_signal_FIRFilter"),
	// Component::constructorDef<CPS::Signal::TurbineGovernor>("_signal_TurbineGovernor"),

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
	Py_INCREF(&Node<CPS::Complex>::type);
	PyModule_AddObject(m, "_dp_Node", (PyObject*) &Node<CPS::Complex>::type);
	Py_INCREF(&Node<CPS::Real>::type);
	PyModule_AddObject(m, "_emt_Node", (PyObject*) &Node<CPS::Real>::type);

	return m;
}
