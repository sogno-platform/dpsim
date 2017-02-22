#ifndef TOPOLOGYREADER_H
#define TOPOLOGYREADER_H

#include <fstream>
#include <vector>
#include <string>
#include "Simulation.h"
#include "Components.h"

typedef struct { 
	std::string type;
	std::string name; 	
	std::string node1;
	std::string node2;
	std::vector<std::string> parameters; 
} Element; 

typedef struct {
	std::string timeStep;
	std::string finalTime;
	std::vector<Element> elements;
} Config;


class TopologyReader {
	private:

	public:
		TopologyReader();
		~TopologyReader();

		/// Parse topology file
		/// @param f Openend stream containing the configuration file.
		/// @param conf Configuration structure that is filled out during parsing.
		/// @return 0 if successfull, nonzero otherwise.
		int readConfig(std::ifstream &f, Config &conf);
		int parseConfig(Config &conf, std::vector<BaseComponent*> &circElements, double &timeStep, double &finalTime);
};




#endif
