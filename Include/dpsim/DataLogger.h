/*********************************************************************************
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

#include <dpsim/Definitions.h>
#include <cps/Attribute.h>
#include <cps/Node.h>

namespace DPsim {

	class DataLogger {

	protected:
		std::ofstream mLogFile;
		Bool mEnabled;

		static std::ostringstream mNullStream;
		static std::ostream& nullStream();

	public:
		DataLogger(Bool enabled = true);
		DataLogger(String name, Bool enabled = true);
		~DataLogger();

		void logDataLine(Real time, Real data);
		void logDataLine(Real time, const Matrix& data);
		void logDataLine(Real time, const MatrixComp& data);

		void logPhasorNodeValues(Real time, const Matrix& data);
		void logEMTNodeValues(Real time, const Matrix& data);

		void setColumnNames(std::vector<String> names);
	};

	class AttributeDataLogger : public DataLogger {
	protected:
		std::map<String, CPS::AttributeBase::Ptr> mAttributes;

	public:
		using Ptr = std::shared_ptr<AttributeDataLogger>;
		using DataLogger::DataLogger;

		void addAttribute(const String &name, CPS::AttributeBase::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<Int>::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<Real>::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<Complex>::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<MatrixVar<Real>>::Ptr attr);
		void addAttribute(const String &name, CPS::Attribute<MatrixVar<Complex>>::Ptr attr);

		template<typename VarType>
		void addNode(typename CPS::Node<VarType>::Ptr node) {
			addAttribute(node->name() + ".voltage", node->attributeMatrix("voltage"));
		}

		void log(Real time);
	};
}

