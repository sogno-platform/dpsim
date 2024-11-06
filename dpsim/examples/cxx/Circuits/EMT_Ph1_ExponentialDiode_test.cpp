#include "../Examples.h"
#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

void EMT_Ph1_ExponentialDiode() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_ExponentialDiode_test";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n1", PhaseType::Single);

  // Components

  auto vs0 = Ph1::VoltageSource::make("vs0");
  vs0->setParameters(CPS::Complex(1., 0.), 50.0);

  auto load = CPS::EMT::Ph1::Resistor::make("Load");
  load->setParameters(10.);

  auto expDiode = Ph1::ExponentialDiode::make("ExponentialDiode");
  expDiode->setParameters(
      0.000001, 0.027); //Calling this is optional. If this method call
                        //is omitted, the diode will get the following
                        //values by default:
                        //I_S = 0.000001 (A) and V_T = 0.027 (V).

  // Topology
  load->connect(SimNode::List{n1, n2});

  expDiode->connect(SimNode::List{
      SimNode::GND,
      n2}); //Looking at the diode's current flow direction, always
            //connect by listing the output/low voltage side first
            //and the input/high voltage side second

  vs0->connect(SimNode::List{SimNode::GND, n1});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{load, vs0, expDiode});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("I_ExponentialDiode", expDiode->attribute("i_intf"));
  logger->logAttribute("V_ExponentialDiode", expDiode->attribute("v_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.doInitFromNodesAndTerminals(false);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::ITERATIVEMNA);
  sim.setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration());
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

int main() {
  EMT_Ph1_ExponentialDiode();
  return 0;
}