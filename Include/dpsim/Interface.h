/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <cps/Attribute.h>
#include <cps/Task.h>

namespace DPsim {

	/**
	 * After an Interface is created, components that should use values
	 * from this interface can be registered with it using the appropiate
	 * methods implemented by the subclass. Subclasses must also implement the
	 * readValues and writeValues methods, which should update the values of
	 * the registered components or send voltages or currents to the external
	 * sink.
	 */
	class Interface {

	public:
		typedef std::shared_ptr<Interface> Ptr;

        Interface() = default;
		virtual ~Interface() { };

		virtual void open(CPS::Logger::Log log) = 0;
		virtual void close() = 0;

		virtual CPS::Attribute<Int>::Ptr importInt(UInt idx) = 0;
		virtual CPS::Attribute<Real>::Ptr importReal(UInt idx) = 0;
		virtual CPS::Attribute<Bool>::Ptr importBool(UInt idx) = 0;
		virtual CPS::Attribute<Complex>::Ptr importComplex(UInt idx) = 0;
		virtual CPS::Attribute<Complex>::Ptr importComplexMagPhase(UInt idx) = 0;

		virtual void exportInt(CPS::Attribute<Int>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="") = 0;
		virtual void exportReal(CPS::Attribute<Real>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="") = 0;
		virtual void exportBool(CPS::Attribute<Bool>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="") = 0;
		virtual void exportComplex(CPS::Attribute<Complex>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="") = 0;

		/** Read data for a timestep from the interface and passes the values
		 * to all registered current / voltage sources.
		 */
		virtual void readValues(bool blocking = true) = 0;

		/** Write all exported values to the interface. Called after every timestep.
		 * @param model Reference to the system model which should be used to
		 * calculate needed voltages.
		 */
		virtual void writeValues() = 0;

		virtual CPS::Task::List getTasks() = 0;
	};
}

