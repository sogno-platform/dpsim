/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <fstream>
#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim/ThreadLevelScheduler.h>

using namespace DPsim;
using namespace CPS;

IdentifiedObject::List multiply_diakoptics(SystemTopology &sys, Int copies,
                                           Real resistance, Real inductance,
                                           Real capacitance, Int splits = 0) {

  sys.multiply(copies);
  int counter = 0;
  std::vector<String> nodes = {"BUS5", "BUS8", "BUS6"};
  IdentifiedObject::List tear_components;
  Int splitEvery = 0;

  if ((splits == 0) || (splits == copies + 1)) {
    splitEvery = 1;
  } else {
    splitEvery = Int(std::round((copies + 1) / splits));
  }
  //std::cout << "split every: " << splitEvery << std::endl;

  for (auto orig_node : nodes) {
    std::vector<String> nodeNames{orig_node};
    for (int i = 2; i < copies + 2; i++) {
      nodeNames.push_back(orig_node + "_" + std::to_string(i));
    }
    nodeNames.push_back(orig_node);

    int nlines = copies == 1 ? 1 : copies + 1;
    for (int i = 0; i < nlines; i++) {
      auto line = DP::Ph1::PiLine::make("line" + std::to_string(counter));
      line->setParameters(resistance, inductance, capacitance);
      line->connect({sys.node<DP::SimNode>(nodeNames[i]),
                     sys.node<DP::SimNode>(nodeNames[i + 1])});

      if (i % splitEvery == 0) {
        sys.addTearComponent(line);
        //std::cout 	<< "add tear line between node " << sys.node<DP::SimNode>(nodeNames[i])->name()
        //			<< " and node " << sys.node<DP::SimNode>(nodeNames[i+1])->name() << std::endl;
      } else
        sys.addComponent(line);

      counter += 1;
    }
  }
  return tear_components;
}

void simulateDiakoptics(std::list<fs::path> filenames, Int copies, Int threads,
                        UInt splits = 0, Int seq = 0) {

  String simName = "WSCC_9bus_diakoptics_" + std::to_string(copies) + "_" +
                   std::to_string(threads) + "_" + std::to_string(splits) +
                   "_" + std::to_string(seq);
  Logger::setLogDir("logs/" + simName);

  CIM::Reader reader(simName, Logger::Level::off, Logger::Level::off);
  SystemTopology sys =
      reader.loadCIM(60, filenames, Domain::DP, PhaseType::Single,
                     CPS::GeneratorType::IdealVoltageSource);

  if (copies > 0)
    IdentifiedObject::List tearComps =
        multiply_diakoptics(sys, copies, 12.5, 0.16, 1e-6, splits);

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setTimeStep(0.0001);
  sim.setFinalTime(0.5);
  sim.setDomain(Domain::DP);
  if (threads > 0)
    sim.setScheduler(std::make_shared<OpenMPLevelScheduler>(threads));
  if (copies > 0)
    sim.setTearingComponents(sys.mTearComponents);

  // Logging
  auto logger = DataLogger::make(simName);
  for (Int cop = 1; cop <= copies; cop++) {
  	for (Int bus  = 1; bus <= 9; bus++) {
  		String attrName = "v" + std::to_string(bus) + "_" + std::to_string(cop);
  		String nodeName = "BUS" + std::to_string(bus) + "_" + std::to_string(cop);
  		if (cop == 1) {
  			attrName = "v" + std::to_string(bus);
  			nodeName = "BUS" + std::to_string(bus);
  		}
  		logger->logAttribute(attrName, sys.node<DP::SimNode>(nodeName)->attribute("v"));
  	}
  }
  sim.addLogger(logger);

  sim.run();
  sim.logStepTimes(simName + "_step_times");
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv);

  std::list<fs::path> filenames;
  filenames = DPsim::Utils::findFiles(
      {"WSCC-09_RX_DI.xml", "WSCC-09_RX_EQ.xml", "WSCC-09_RX_SV.xml",
       "WSCC-09_RX_TP.xml"},
      "../../../build/_deps/cim-data-src/WSCC-09/WSCC-09_RX", "CIMPATH");

  //for (Int copies = 0; copies < 10; copies++) {
  //	for (Int threads = 0; threads <= 12; threads = threads+2) {
  //		for (Int splits = 0; splits < copies; splits++)
  //			simulateDiakoptics(filenames, copies, threads, splits);
  //	}
  //}
  //simulateDiakoptics(filenames, 19, 8, 0);

  Int numCopies = 0;
  Int numThreads = 0;
  Int numSeq = 0;
  Int numSplits = 0;

  if (args.options.find("copies") != args.options.end())
    numCopies = args.getOptionInt("copies");
  if (args.options.find("threads") != args.options.end())
    numThreads = args.getOptionInt("threads");
  if (args.options.find("seq") != args.options.end())
    numSeq = args.getOptionInt("seq");
  if (args.options.find("splits") != args.options.end())
    numSplits = args.getOptionInt("splits");

  std::cout << "Simulate with " << numCopies << " copies, " << numThreads
            << " threads, " << numSplits << " splits, sequence number "
            << numSeq << std::endl;
  simulateDiakoptics(filenames, numCopies, numThreads, numSplits, numSeq);
}
