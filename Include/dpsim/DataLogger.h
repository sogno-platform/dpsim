/**
 * @file
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

#include <map>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <cps/PtrFactory.h>
#include <cps/Attribute.h>
#include <cps/Node.h>
#include <cps/Task.h>

namespace DPsim {

	class DataLogger : public SharedFactory<DataLogger> {

	protected:
		std::ofstream mLogFile;
		String mName;
		Bool mEnabled;
		UInt mDownsampling;
		fs::path mFilename;

		std::map<String, CPS::AttributeBase::Ptr> mAttributes;

		void logDataLine(Real time, Real data);
		void logDataLine(Real time, const Matrix& data);
		void logDataLine(Real time, const MatrixComp& data);

	public:
		typedef std::shared_ptr<DataLogger> Ptr;
		typedef std::vector<DataLogger::Ptr> List;

		DataLogger(Bool enabled = true);
		DataLogger(String name, Bool enabled = true, UInt downsampling = 1);

		void open();
		void close();
		void reopen() {
			close();
			open();
		}

		void logPhasorNodeValues(Real time, const Matrix& data, Int freqNum = 1);
		void logEMTNodeValues(Real time, const Matrix& data);

		void setColumnNames(std::vector<String> names);

		void addAttribute(const String &name, CPS::AttributeBase::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<Int>::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<Real>::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<Complex>::Ptr attr);
		void addAttribute(const String &name, CPS::MatrixRealAttribute::Ptr attr);
		void addAttribute(const String &name, CPS::MatrixCompAttribute::Ptr attr, UInt rowsMax = 0, UInt colsMax = 0);

		template<typename VarType>
		void addNode(typename CPS::Node<VarType>::Ptr node) {
			addAttribute(node->name() + ".voltage", node->attributeMatrix("voltage"));
		}

		void log(Real time, Int timeStepCount);

		CPS::Task::Ptr getTask();

		class Step : public CPS::Task {
		public:
			Step(DataLogger& logger) :
				Task(logger.mName + ".Write"), mLogger(logger) {
				for (auto attr : logger.mAttributes) {
					mAttributeDependencies.push_back(attr.second);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			DataLogger& mLogger;
		};
	};
}

