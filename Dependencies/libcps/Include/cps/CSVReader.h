/**
 *
 * @file
 * @author  Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * 			Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2019, Institute for Automation of Complex Power Systems, EONERC
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
#include <cps/Logger.h>
#include <cps/SystemTopology.h>
#include <cps/SP/SP_Ph1_Load.h>
#include <cps/DP/DP_Ph1_PQLoadCS.h>
#include <cps/DP/DP_Ph1_AvVoltageSourceInverterDQ.h>
#include <cps/SP/SP_Ph1_AvVoltageSourceInverterDQ.h>

namespace CPS {
	/// reads load profiles (csv files only) and assign them to the corresponding load object
	class CSVReader {
	private:
		/// Logger
		Logger::Log mSLog;
		/// path of load profile files (csv file)
		String mPath;
		/// list of load profile files with path
		std::list<std::experimental::filesystem::path> mFileList;
		/// assign pattern, used when the MANUAL mode is selected
		std::map <String, String> mAssignPattern;

	public:
		/// set load profile assigning pattern. AUTO for assigning load profile name (csv file name) to load object with the same name (mName)
		/// MANUAL for providing an assign pattern manually. see power flow example: CIM/CIGRE_MV_PowerFlowTest_LoadProfiles.cpp
		enum class Mode { AUTO, MANUAL };

		/*
		 Time Stamp Format.
         HHMMSS:  Hours : Minutes : Seconds, it be casted to the corresponding SECONDS.
         SECONDS: profiles recorded with total seconds.
         PVGEN: format comply with https://www.fein-aachen.org/projects/PVgenerator/
		*/
		enum class DataFormat { HHMMSS, SECONDS, HOURS, MINUTES };

		///
		CSVReader(String name, std::list<std::experimental::filesystem::path> path, Logger::Level logLevel);
		///
		CSVReader(String name, String path, Logger::Level logLevel);
		///
		CSVReader(String name, std::list<std::experimental::filesystem::path> path, std::map<String, String>& assignList, Logger::Level logLevel);
		///
		CSVReader(String name, String path, std::map<String, String>& assignList, Logger::Level logLevel);

		///	convert HH:MM:SS format timestamp into total seconds.
		///	e.g.: 00 : 01 : 00 -- > 60.
		Real time_format_convert(const String& time);


		std::vector<PQData> readLoadProfileDP(std::experimental::filesystem::path file,
			Real start_time = -1, Real time_step = 1, Real end_time = -1, Real scale_factor= 1,
			CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);

		// void assignLoadProfilePF(std::vector<std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ>>& loads,
		// 	Real start_time = -1, Real time_step = 1, Real end_time = -1, Real scale_factor= 1,
		// 	CSVReader::Mode mode = CSVReader::Mode::AUTO,
		// 	CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);

		void assignLoadProfileSP(std::vector<std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ>>& loads,
			Real start_time = -1, Real time_step = 1, Real end_time = -1, Real scale_factor= 1,
			CSVReader::Mode mode = CSVReader::Mode::AUTO,
			CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);

		void assignLoadProfileDP(std::vector<std::shared_ptr<CPS::DP::Ph1::AvVoltageSourceInverterDQ>>& loads,
			Real start_time = -1, Real time_step = 1, Real end_time = -1, Real scale_factor= 1,
			CSVReader::Mode mode = CSVReader::Mode::AUTO,
			CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);

		/// TODO : deprecate in the future
		/// read in load profile with time stamp format specified
		PowerProfile readLoadProfile(std::experimental::filesystem::path file,
			Real start_time = -1, Real time_step = 1, Real end_time = -1,
			CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);
		///
		std::vector<Real> readPQData (std::experimental::filesystem::path file,
			Real start_time = -1, Real time_step = 1, Real end_time = -1,
			CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);
		/// assign load profile to corresponding load object
		void assignLoadProfile(SystemTopology& sys,
			Real start_time = -1, Real time_step = 1, Real end_time = -1,
			CSVReader::Mode mode = CSVReader::Mode::AUTO,
			CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);
		///
		void assignPVGeneration(SystemTopology& sys,
			Real start_time = -1, Real time_step = 1, Real end_time = -1,
			CSVReader::Mode mode = CSVReader::Mode::AUTO);

		/// interpolation for PQ data points
		PQData interpol_linear(std::map<Real, PQData>& data_PQ, Real x);

		/// interpolation for weighting factor data points
		Real interpol_linear(std::map<Real, Real>& data_wf, Real x);
	};


	// #### csv reader section
	class CSVRow {
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

	class CSVReaderIterator {
	public:
		CSVReaderIterator(std::istream& str);
		CSVReaderIterator();

		CSVReaderIterator& next();
		CSVReaderIterator next(Int);
		CSVReaderIterator& step(Int time_step);
		CSVRow const& operator*() const { return m_row; };
		Bool operator==(CSVReaderIterator const& rhs) {
			return ((this == & rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
		}
		Bool operator!=(CSVReaderIterator const&rhs) {
			return !((*this) == rhs);
		}
	private:
		std::istream* m_str;
		CSVRow m_row;
	};
}

