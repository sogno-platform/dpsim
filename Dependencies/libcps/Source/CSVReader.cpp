/**
 *
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * 			Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include <cps/CSVReader.h>

namespace fs = std::experimental::filesystem;

using namespace CPS;

void CSVRow::readNextRow(std::istream& str) {
	std::string line;
	std::getline(str, line);

	std::stringstream   lineStream(line);
	std::string         cell;

	m_data.clear();
	while (std::getline(lineStream >> std::ws, cell, ',')) {
		m_data.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!lineStream && cell.empty()){
		// If there was a trailing comma then add an empty element.
		m_data.push_back("");
	}
}

int CSVRow::size() const {
	return m_data.size();
}

CSVReaderIterator& CSVReaderIterator::next() {
	if (m_str) {
		m_row.readNextRow(*m_str);
		if (!(*m_str)) {
			m_str = NULL;
		}
	}
	return *this;
}

CSVReaderIterator CSVReaderIterator::next(int) {
	CSVReaderIterator tmp(*this);
	this->next();
	return tmp;
}

CSVReaderIterator& CSVReaderIterator::step(int time_step) {
	while (time_step != 0) {
		if (m_str) {
			m_row.readNextRow(*m_str);
			if (!(*m_str)) {
				m_str = NULL;
			}
		}
		time_step -= 1;
	}
	return *this;
}


CSVReaderIterator::CSVReaderIterator(std::istream& str)
	: m_str(str.good() ? &str : NULL) {
	this->next();
}

CSVReaderIterator::CSVReaderIterator():m_str(NULL) { }

CSVReader::CSVReader(CPS::String name, std::list<fs::path> paths, CPS::Logger::Level logLevel) {
	mSLog = Logger::get(name + "_csvReader", logLevel);
	//mFileList = paths;
	for(auto file : paths){
		if(file.string().find(".csv")!=std::string::npos){
				mFileList.push_back(file);
				std::cout<<"add "<< file<<std::endl;
		}
	}

}

CSVReader::CSVReader(CPS::String name, CPS::String path,  CPS::Logger::Level logLevel) {
	mSLog = Logger::get(name + "_csvReader", logLevel);

	mPath = path;
	for (const auto & entry : std::experimental::filesystem::directory_iterator(path)) {
			mFileList.push_back(entry.path());

	}
}

CSVReader::CSVReader(CPS::String name, CPS::String path, std::map<String, String>& assignList, CPS::Logger::Level logLevel)
	: CSVReader(name, path, logLevel) {

	mAssignPattern = assignList;
}

CSVReader::CSVReader(CPS::String name, std::list<fs::path> paths, std::map<String, String>& assignList,
CPS::Logger::Level logLevel): CSVReader(name, paths, logLevel){
	mAssignPattern = assignList;
}

CPS::Real CSVReader::time_format_convert(const CPS::String& time) {
	int hh, mm, ss = 0;
	CPS::Real secs = 0;
	if (sscanf(time.c_str(), "%d:%d:%d", &hh, &mm, &ss) >= 2) {
		secs = hh * 3600 + mm * 60 + ss;
	}
	return secs;
}

std::vector<PQData> CSVReader::readLoadProfileDP(std::experimental::filesystem::path file,
	Real start_time, Real time_step, Real end_time, Real scale_factor, CSVReader::DataFormat format) {

	std::vector<PQData> load_profileDP;
	std::ifstream csvfile(file);
	CSVReaderIterator row_(csvfile);

	// ignore the first row if it is a title
	if (!std::isdigit((*row_).get(0)[0])) {
		row_.next();
	}
	/*
	 loop over rows of the csv file to find the entry point to read in.
	 if start_time and end_time are negative (as default), it reads in all rows.
	*/
	Real presentTime = 0;
	for (; row_ != CSVReaderIterator(); row_.next()) {
		if ((start_time < 0) | (Int(presentTime) + 1 > Int(start_time)))
			break;
		presentTime++;
	}
	/*
	 reading data after entry point until end_time is reached
	*/
	for (; row_ != CSVReaderIterator(); row_.next()) {
		// IMPORTANT: take care of units. assume kW
		PQData pq;
		// multiplied by 1000 due to unit conversion (kw to w)
		pq.p = std::stod((*row_).get(1)) * 1000 * scale_factor;
		pq.q = std::stod((*row_).get(2)) * 1000 * scale_factor;
		load_profileDP.push_back(pq);
		if (end_time > 0 && presentTime > end_time)
			break;
		/*while (presentTime + time_step < Int(presentTime) + 1)
		{
			presentTime += time_step;
			p_data.push_back(p_data.back());
		}*/
		presentTime = Int(presentTime) + 1;
	}
	std::cout<<"CSV loaded."<<std::endl;
	return load_profileDP;
}

void CSVReader::assignLoadProfileSP(std::vector<std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ>>& loads,
 Real start_time, Real time_step, Real end_time, Real scale_factor,
	CSVReader::Mode mode, CSVReader::DataFormat format) {

	switch (mode) {
		case CSVReader::Mode::AUTO: {
			for (auto load : loads) {
				if(!load->isLoad())
					continue;
				String load_name = load->name();
				for (auto file : mFileList) {
					String file_name = file.filename().string();
					/// changing file name and load name to upper case for later matching
					for (auto & c : load_name) c = toupper(c);
					for (auto & c : file_name) c = toupper(c);
					/// strip off all non-alphanumeric characters
					load_name.erase(remove_if(load_name.begin(), load_name.end(), [](char c) { return !isalnum(c); }), load_name.end());
					file_name.erase(remove_if(file_name.begin(), file_name.end(), [](char c) { return !isalnum(c); }), file_name.end());
					if (std::stoi(file_name) == std::stoi(load_name)) {
						load->mLoadProfile = readLoadProfileDP(file, start_time, time_step, end_time, scale_factor, format);
						mSLog->info("Assigned {} to {}", file.filename().string(), load->name());
					}
				}
			}

			break;
		}
		case CSVReader::Mode::MANUAL: {
			Int LP_assigned_counter = 0;
			Int LP_not_assigned_counter = 0;
			mSLog->info("Assigning load profiles with user defined pattern ...");
			for (auto load : loads) {
					std::map<String, String>::iterator file = mAssignPattern.find(load->name());
					if (file == mAssignPattern.end()) {

						mSLog->info("{} has no profile given.", load->name());
						LP_not_assigned_counter++;
						continue;
					}
					for(auto path: mFileList){
						if(path.string().find(file->second)!= std::string::npos){
							load->mLoadProfile = readLoadProfileDP(path, start_time, time_step, end_time, scale_factor);
							mSLog->info("Assigned {}.csv to {}", file->second, load->name());
							LP_assigned_counter++;
						}

					}
			}
			mSLog->info("Assigned profiles for {} loads, {} not assigned.", LP_assigned_counter, LP_not_assigned_counter);
			break;
		}
		default: {
			throw std::invalid_argument(
				"Load profile assign mode error");
			break;
		}
	}
}


// void CSVReader::assignLoadProfilePF(std::vector<std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ>>& loads,
//  Real start_time, Real time_step, Real end_time, Real scale_factor,
// 	CSVReader::Mode mode, CSVReader::DataFormat format) {

// 	switch (mode) {
// 		case CSVReader::Mode::AUTO: {
// 			for (auto load : loads) {
// 				if(!load->isLoad())
// 					continue;
// 				String load_name = load->name();
// 				for (auto file : mFileList) {
// 					String file_name = file.filename().string();
// 					/// changing file name and load name to upper case for later matching
// 					for (auto & c : load_name) c = toupper(c);
// 					for (auto & c : file_name) c = toupper(c);
// 					/// strip off all non-alphanumeric characters
// 					load_name.erase(remove_if(load_name.begin(), load_name.end(), [](char c) { return !isalnum(c); }), load_name.end());
// 					file_name.erase(remove_if(file_name.begin(), file_name.end(), [](char c) { return !isalnum(c); }), file_name.end());
// 					if (std::stoi(file_name) == std::stoi(load_name)) {
// 						load->mPFAvVoltageSourceInverter->mLoadProfile = readLoadProfile(std::experimental::filesystem::path(mPath + file->second + ".csv"), start_time, time_step, end_time);
// 						load->mPFAvVoltageSourceInverter->use_profile = true;
// 						mSLog->info("Assigned {} to {}", file.filename().string(), load->name());
// 					}
// 				}
// 			}

// 			break;
// 		}
// 		case CSVReader::Mode::MANUAL: {
// 			Int LP_assigned_counter = 0;
// 			Int LP_not_assigned_counter = 0;
// 			mSLog->info("Assigning load profiles with user defined pattern ...");
// 			for (auto load : loads) {
// 					std::map<String, String>::iterator file = mAssignPattern.find(load->name());
// 					if (file == mAssignPattern.end()) {

// 						mSLog->info("{} has no profile given.", load->name());
// 						LP_not_assigned_counter++;
// 						continue;
// 					}
// 					for(auto path: mFileList){
// 						if(path.string().find(file->second)!= std::string::npos){
// 							load->mPFAvVoltageSourceInverter->mLoadProfile = readLoadProfile(std::experimental::filesystem::path(mPath + file->second + ".csv"), start_time, time_step, end_time);
// 							load->mPFAvVoltageSourceInverter->use_profile = true;
// 							mSLog->info("Assigned {}.csv to {}", file->second, load->name());
// 							LP_assigned_counter++;
// 						}

// 					}
// 			}
// 			mSLog->info("Assigned profiles for {} loads, {} not assigned.", LP_assigned_counter, LP_not_assigned_counter);
// 			break;
// 		}
// 		default: {
// 			throw std::invalid_argument(
// 				"Load profile assign mode error");
// 			break;
// 		}
// 	}
// }


void CSVReader::assignLoadProfileDP(std::vector<std::shared_ptr<CPS::DP::Ph1::AvVoltageSourceInverterDQ>>& loads,
 Real start_time, Real time_step, Real end_time, Real scale_factor,
	CSVReader::Mode mode, CSVReader::DataFormat format) {

	switch (mode) {
		case CSVReader::Mode::AUTO: {
			for (auto load : loads) {
				if(!load->isLoad())
					continue;
				String load_name = load->name();
				for (auto file : mFileList) {
					String file_name = file.filename().string();
					/// changing file name and load name to upper case for later matching
					for (auto & c : load_name) c = toupper(c);
					for (auto & c : file_name) c = toupper(c);
					/// strip off all non-alphanumeric characters
					load_name.erase(remove_if(load_name.begin(), load_name.end(), [](char c) { return !isalnum(c); }), load_name.end());
					file_name.erase(remove_if(file_name.begin(), file_name.end(), [](char c) { return !isalnum(c); }), file_name.end());
					if (std::stoi(file_name) == std::stoi(load_name)) {
						load->mLoadProfile = readLoadProfileDP(file, start_time, time_step, end_time, scale_factor, format);
						mSLog->info("Assigned {} to {}", file.filename().string(), load->name());
					}
				}
			}

			break;
		}
		case CSVReader::Mode::MANUAL: {
			Int LP_assigned_counter = 0;
			Int LP_not_assigned_counter = 0;
			mSLog->info("Assigning load profiles with user defined pattern ...");
			for (auto load : loads) {
					std::map<String, String>::iterator file = mAssignPattern.find(load->name());
					if (file == mAssignPattern.end()) {

						mSLog->info("{} has no profile given.", load->name());
						LP_not_assigned_counter++;
						continue;
					}
					for(auto path: mFileList){
						if(path.string().find(file->second)!= std::string::npos){
							load->mLoadProfile = readLoadProfileDP(path, start_time, time_step, end_time, scale_factor);
							mSLog->info("Assigned {}.csv to {}", file->second, load->name());
							LP_assigned_counter++;
						}

					}
			}
			mSLog->info("Assigned profiles for {} loads, {} not assigned.", LP_assigned_counter, LP_not_assigned_counter);
			break;
		}
		default: {
			throw std::invalid_argument(
				"Load profile assign mode error");
			break;
		}
	}
}




PowerProfile CSVReader::readLoadProfile(std::experimental::filesystem::path file,
	Real start_time, Real time_step, Real end_time, CSVReader::DataFormat format) {

	PowerProfile load_profile;
	std::ifstream csvfile(file);
	bool need_that_conversion = (format == DataFormat::HHMMSS) ? true : false;
	bool data_with_weighting_factor = false;

	CSVReaderIterator loop(csvfile);

	// ignore the first row if it is a title
	if (!std::isdigit((*loop).get(0)[0])) {
		loop.next();
	}
	/*
	 loop over rows of the csv file to find the entry point to read in.
	 and determine data type prior to read in. (assuming only time,p,q or time,weighting factor)
	 if start_time and end_time are negative (as default), it reads in all rows.
	*/
	for (; loop != CSVReaderIterator(); loop.next()) {
		CSVReaderIterator nextRow = loop;
		nextRow.next();
		CPS::Real nextTime = (need_that_conversion) ? time_format_convert((*nextRow).get(0)) : std::stod((*nextRow).get(0));
		if ((*nextRow).size() == 2) {
			data_with_weighting_factor = true;
		}
		if ((start_time < 0) | (nextTime >= Int(start_time)))
		{
			break;
		}
	}
	/*
	 reading data after entry point until end_time is reached
	*/
	for (; loop != CSVReaderIterator(); loop.next()) {
		CPS::Real currentTime = (need_that_conversion) ? time_format_convert((*loop).get(0)) : std::stod((*loop).get(0));
		if (data_with_weighting_factor) {
			Real wf = std::stod((*loop).get(1));
			load_profile.weightingFactors.insert(std::pair<Real, Real>(currentTime, wf));
		}
		else {
		PQData pq;
		// multiplied by 1000 due to unit conversion (kw to w)
		pq.p = std::stod((*loop).get(1)) * 1000;
		pq.q = std::stod((*loop).get(2)) * 1000;
		load_profile.pqData.insert(std::pair<Real,PQData>(currentTime,pq));
		}

		if (end_time > 0 && currentTime > end_time)
			break;
	}
		std::vector<CPS::Real> times;
	for (CPS::Real x_ = start_time; x_ <= end_time; x_ += time_step) {
		times.push_back(x_);
	}

	for (auto x : times) {
		if (load_profile.pqData.find(x) == load_profile.pqData.end()) {
			if (data_with_weighting_factor) {
				Real y = interpol_linear(load_profile.weightingFactors, x);
				load_profile.weightingFactors.insert(std::pair<Real, Real>(x, y));
			}
			else {
				PQData y = interpol_linear(load_profile.pqData, x);
				load_profile.pqData.insert(std::pair<Real,PQData>(x,y));
			}
		}
	}

	return load_profile;
}

// can only read one file for now
std::vector<Real> CSVReader::readPQData(fs::path file,
	Real start_time, Real time_step, Real end_time,
	CSVReader::DataFormat format) {

	std::vector<Real> p_data;
	std::ifstream csvfile(file);
	CSVReaderIterator row_(csvfile);

	// ignore the first row if it is a title
	if (!std::isdigit((*row_).get(0)[0])) {
		row_.next();
	}
	/*
	 loop over rows of the csv file to find the entry point to read in.
	 if start_time and end_time are negative (as default), it reads in all rows.
	*/
	Real presentTime = 0;
	for (; row_ != CSVReaderIterator(); row_.next()) {
		if ((start_time < 0) | (Int(presentTime) + 1 > Int(start_time)))
			break;
		presentTime++;
	}
	/*
	 reading data after entry point until end_time is reached
	*/
	for (; row_ != CSVReaderIterator(); row_.next()) {
		// IMPORTANT: take care of units. assume kW
		p_data.push_back(std::stod((*row_).get(0)) * 1000);
		if (end_time > 0 && presentTime > end_time)
			break;
		/*while (presentTime + time_step < Int(presentTime) + 1)
		{
			presentTime += time_step;
			p_data.push_back(p_data.back());
		}*/
		presentTime = Int(presentTime) + 1;
	}
	std::cout<<"CSV loaded."<<std::endl;
	return p_data;
}

void CSVReader::assignLoadProfile(CPS::SystemTopology& sys, Real start_time, Real time_step, Real end_time,
	CSVReader::Mode mode, CSVReader::DataFormat format) {

	switch (mode) {
		case CSVReader::Mode::AUTO: {
			for (auto obj : sys.mComponents) {
				if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(obj)) {
					mSLog->info("Comparing csv file names with load mRIDs ...");
					String load_name = load->name();
					for (auto file : mFileList) {
						String file_name = file.filename().string();
						/// changing file name and load name to upper case for later matching
						for (auto & c : load_name) c = toupper(c);
						for (auto & c : file_name) c = toupper(c);
						/// strip off all non-alphanumeric characters
						load_name.erase(remove_if(load_name.begin(), load_name.end(), [](char c) { return !isalnum(c); }), load_name.end());
						file_name.erase(remove_if(file_name.begin(), file_name.end(), [](char c) { return !isalnum(c); }), file_name.end());
						if (std::string(file_name.begin(), file_name.end() - 3).compare(load_name) == 0) {
							load->mLoadProfile = readLoadProfile(file, start_time, time_step, end_time, format);
							load->use_profile = true;
							mSLog->info("Assigned {} to {}", file.filename().string(), load->name());
						}
					}
				}
			}
			break;
		}
		case CSVReader::Mode::MANUAL: {
			Int LP_assigned_counter = 0;
			Int LP_not_assigned_counter = 0;
			mSLog->info("Assigning load profiles with user defined pattern ...");
			for (auto obj : sys.mComponents) {
				if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(obj)) {
					std::map<String, String>::iterator file = mAssignPattern.find(load->name());
					if (file == mAssignPattern.end()) {
						std::cout<< load->name()<<" has no profile given."<<std::endl;
						mSLog->info("{} has no profile given.", load->name());
						LP_not_assigned_counter++;
						continue;
					}
					load->mLoadProfile = readLoadProfile(std::experimental::filesystem::path(mPath + file->second + ".csv"), start_time, time_step, end_time);
					load->use_profile = true;
					std::cout<<" Assigned "<< file->second<< " to " <<load->name()<<std::endl;
					mSLog->info("Assigned {}.csv to {}", file->second, load->name());
					LP_assigned_counter++;
				}
			}
			mSLog->info("Assigned profiles for {} loads, {} not assigned.", LP_assigned_counter, LP_not_assigned_counter);
			break;
		}
		default: {
			throw std::invalid_argument(
				"Load profile assign mode error");
			break;
		}
	}
}

CPS::PQData CSVReader::interpol_linear(std::map<CPS::Real, CPS::PQData>& pqData, CPS::Real x) {
	std::map <Real, PQData>::const_iterator entry = pqData.upper_bound(x);
	PQData y;

	if (entry == pqData.end()) {
		return (--entry)->second;
	}
	if (entry == pqData.begin()) {
		return entry->second;
	}
	std::map <Real, PQData>::const_iterator prev = entry;
	--prev;

	const CPS::Real delta = (x - prev->first) / (entry->first - prev->first);

	y.p = delta * entry->second.p + (1 - delta)*prev->second.p;
	y.q = delta * entry->second.q + (1 - delta)*prev->second.q;
	return y;
}

CPS::Real CSVReader::interpol_linear(std::map<CPS::Real, CPS::Real>& weightingFactors, CPS::Real x) {
	std::map <Real, Real>::const_iterator entry = weightingFactors.upper_bound(x);
	CPS::Real y;

	if (entry == weightingFactors.end()) {
		return (--entry)->second;
	}
	if (entry == weightingFactors.begin()) {
		return entry->second;
	}
	std::map <Real, Real>::const_iterator prev = entry;
	--prev;

	const CPS::Real delta = (x - prev->first) / (entry->first - prev->first);

	y = delta * entry->second + (1 - delta)*prev->second;
	return y;
}






