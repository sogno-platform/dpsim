#include "TopologyReader.h"
#include "Components.h"

TopologyReader::TopologyReader() {

}

TopologyReader::~TopologyReader() {

}

/// Parse topology file
/// @param f Openend stream containing the configuration file.
/// @param conf Configuration structure that is filled out during parsing.
/// @return 0 if successfull, nonzero otherwise.

int TopologyReader::readConfig(std::ifstream &f, Config &conf) {
	std::string s;
		
	getline(f, s);

	if (!s.empty() && s[0] == '#')
		getline(f, s);

	if (conf.timeStep.empty()) {
		std::string::size_type p = s.find(',');
		conf.timeStep = s.substr(0, p);
		s.erase(0, p + 1);
	}

	if (conf.finalTime.empty()) {
		std::string::size_type p = s.find(',');
		conf.finalTime = s.substr(0, p);
		s.erase(0, p + 1);
	}

	for (int line = 1; f.good(); line++) {
		
		getline(f, s);

		if (!s.empty() && s[0] == '#')
			continue;

		Element element;

		for (int elementIdx = 1; s.find(',') != std::string::npos; elementIdx++) {
			if (element.type.empty()) {
				std::string::size_type p = s.find(',');				
				element.type = s.substr(0, p);
				s.erase(0, p + 1);
			}
			else if (element.name.empty()) {
				std::string::size_type p = s.find(',');			
				element.name = s.substr(0, p);
				s.erase(0, p + 1);
			}
			else if (element.node1.empty()) {
				std::string::size_type p = s.find(',');			
				element.node1 = s.substr(0, p);
				s.erase(0, p + 1);
			}
			else if (element.node2.empty()) {
				std::string::size_type p = s.find(',');			
				element.node2 = s.substr(0, p);
				s.erase(0, p + 1);
			}
			else {
				std::string::size_type p = s.find(',');		
				element.parameters.push_back(s.substr(0, p));
				s.erase(0, p + 1);
			}
		}

		conf.elements.push_back(element);
	}
	return 0;
}


/// Use the parsed configuration structure to create circuit elements

int TopologyReader::parseConfig(Config &conf, std::vector<CircuitElement*> &circElements, double &timeStep, double &finalTime) {

	timeStep = std::stod(conf.timeStep);
	finalTime = std::stod(conf.finalTime);
		
	for (std::vector<Element>::iterator it = conf.elements.begin(); it != conf.elements.end(); ++it) {
		
		CircuitElement* tmpCircElement;

		if (it->type.compare("VoltageSourceWithResistance") == 0) {
			tmpCircElement = new VoltageSourceWithResistance(it->name, std::stoi(it->node1), std::stoi(it->node2), std::stod(it->parameters[0]), std::stod(it->parameters[1]), std::stod(it->parameters[2]));
		}
		else if (it->type.compare("Inductor") == 0) {
			tmpCircElement = new Inductor(it->name, std::stoi(it->node1), std::stoi(it->node2), std::stod(it->parameters[0]));
		}
		else if (it->type.compare("Capacitor") == 0) {
			tmpCircElement = new Capacitor(it->name, std::stoi(it->node1), std::stoi(it->node2), std::stod(it->parameters[0]));
		}
		else if (it->type.compare("LinearResistor") == 0) {
			tmpCircElement = new LinearResistor(it->name, std::stoi(it->node1), std::stoi(it->node2), std::stod(it->parameters[0]));
		}
		else if (it->type.compare("InterfacedInductor") == 0) {
			tmpCircElement = new InterfacedInductor(it->name, std::stoi(it->node1), std::stoi(it->node2), std::stod(it->parameters[0]));
		}
		else {

		}
		circElements.push_back(tmpCircElement);
	}

	return 0;
}