/**
 *
 * @file
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>
#include <experimental/filesystem>
#include <cps/SP/SP_Ph1_Load.h>
#include <cps/Logger.h>
#include <cps/SystemTopology.h>
#include <cps/PowerProfile.h>

namespace CPS {
	/// reads load profiles (csv files only) and assign them to the corresponding load object
	class LoadProfileReader {
	private:
		/// Logger
		Logger::Log mSLog;
		/// path of load profile files (csv file)
		String mPath;
		/// list of load profile files with path
		std::vector<std::experimental::filesystem::path> mFileList;
		/// assign pattern, used when the MANUAL mode is selected
		std::map <String, String> mAssignPattern;

	public:
		/// set load profile assigning pattern. AUTO for assigning load profile name (csv file name) to load object with the same name (mName)
		/// MANUAL for providing an assign pattern manually. see power flow example: CIM/CIGRE_MV_PowerFlowTest_LoadProfiles.cpp
		enum class Mode { AUTO, MANUAL };

		/// time stamp format. HHMMSS for Hours : Minutes : Seconds, it be casted to the corresponding SECONDS. SECONDS for profiles recorded with seconds.
		enum class DataFormat { HHMMSS, SECONDS };

		LoadProfileReader(String name, String path, Logger::Level logLevel);
		LoadProfileReader(String name, String path, std::map<String, String>& assignList, Logger::Level logLevel);

		///	convert HH:MM:SS format timestamp into total seconds.
		///	e.g.: 00 : 01 : 00 -- > 60.
		Real time_format_convert(const String& time);

		/// read in load profile with time stamp format specified
		PowerProfile read(fs::path file,
			Real start_time = -1, Real time_step = 1, Real end_time = -1,
			LoadProfileReader::DataFormat format = LoadProfileReader::DataFormat::SECONDS);

		/// assign load profile to corresponding load object
		void assign(SystemTopology& sys,
			Real start_time = -1, Real time_step = 1, Real end_time = -1,
			LoadProfileReader::Mode mode = LoadProfileReader::Mode::AUTO,
			LoadProfileReader::DataFormat format = LoadProfileReader::DataFormat::SECONDS);

		/// interpolation for PQ data points
		PQData interpol_linear(std::map<Real, PQData>& data_PQ, Real x);

		/// interpolation for weighting factor data points
		Real interpol_linear(std::map<Real, Real>& data_wf, Real x);
	};


	// #### csv reader section
	class LoadProfileRow {
	public:
		///
		String const& get(std::size_t index) const { return m_data[index]; }
		///
		Int size() const;
		///
		void readNextRow(std::istream& str);
	private:
		///
		std::vector<String> m_data;
	};

	class LoadProfileReaderIterator {
	public:
		LoadProfileReaderIterator(std::istream& str);
		LoadProfileReaderIterator();

		LoadProfileReaderIterator& next();
		LoadProfileReaderIterator next(Int);
		LoadProfileReaderIterator& step(Int time_step);
		LoadProfileRow const& operator*() const { return m_row; };
		Bool operator==(LoadProfileReaderIterator const& rhs) {
			return ((this == & rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
		}
		Bool operator!=(LoadProfileReaderIterator const&rhs) {
			return !((*this) == rhs);
		}
	private:
		std::istream* m_str;
		LoadProfileRow m_row;
	};
}

