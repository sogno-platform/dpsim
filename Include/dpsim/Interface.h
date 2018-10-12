/** External interface
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#pragma once

#include <vector>

#include <villas/sample.h>
#include <villas/shmem.h>

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <cps/Attribute.h>
#include <cps/PtrFactory.h>

namespace DPsim {

	/**
	 * After an Interface is created, components that should use values
	 * from this interface can be registered with it using the appropiate
	 * methods implemented by the subclass. Subclasses must also implement the
	 * readValues and writeValues methods, which should update the values of
	 * the registered components or send voltages or currents to the external
	 * sink.
	 */
	class Interface : public SharedFactory<Interface> {

	public:
		typedef std::shared_ptr<Interface> Ptr;
		typedef struct ::sample Sample;
		typedef struct ::shmem_conf Config;
		typedef struct ::shmem_int ShmemInterface;

	protected:
		// Using std::function / lambda makes the other template code nicer, but from
		// the outside, only the attribute-based functions should be used to
		// guarantee proper scheduling

		void addImport(std::function<void(Sample*)> l) { mImports.push_back(l); }
		void addExport(std::function<void(Sample*)> l) { mExports.push_back(l); }

		std::vector<std::function<void(Sample*)>> mExports, mImports;
		CPS::AttributeBase::List mExportAttrs, mImportAttrs;

		ShmemInterface mShmem;
		Sample *mLastSample;

		bool mOpened;
		int mSequence;
		String mRName, mWName;
		Config mConf;

	public:

		/** Create a Interface using the given shmem object names.
		 *
		 * @param wname The name of the POSIX shmem object where samples will be written to.
		 * @param rname The name of the POSIX shmem object where samples will be read from.
		 */
		Interface(const String &wn, const String &rn) :
			mOpened(false),
			mRName(rn),
			mWName(wn)
		{
			mConf.queuelen = 512;
			mConf.samplelen = 64;
			mConf.polling = 0;
		}

		/** Create a Interface with a specific configuration for the output queue.
		 *
		 * @param conf The configuration object for the output queue (see VILLASnode's documentation).
		 */
		Interface(const String &wn, const String &rn, Config *conf) :
			mOpened(false),
			mRName(rn),
			mWName(wn),
			mConf(*conf)
		{ }

		~Interface() {
			if (mOpened)
				close();
		}

		void open();
		void close();

		CPS::Attribute<Int>::Ptr importInt(Int idx);
		CPS::Attribute<Real>::Ptr importReal(Int idx);
		CPS::Attribute<Bool>::Ptr importBool(Int idx);
		CPS::Attribute<Complex>::Ptr importComplex(Int idx);

		void addExport(CPS::Attribute<Int>::Ptr attr, Int idx);
		void addExport(CPS::Attribute<Real>::Ptr attr, Int idx);
		void addExport(CPS::Attribute<Bool>::Ptr attr, Int idx);
		void addExport(CPS::Attribute<Complex>::Ptr attr, Int idx);

		/** Read data for a timestep from the interface and passes the values
		 * to all registered current / voltage sources.
		 */
		void readValues(bool blocking = true);

		/** Write all exported values to the interface. Called after every timestep.
		 * @param model Reference to the system model which should be used to
		 * calculate needed voltages.
		 */
		void writeValues();
	};
}

