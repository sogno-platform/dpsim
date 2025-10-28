#include "../Examples.h"
#include "../GeneratorFactory.h"
#include <DPsim.h>

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
const Examples::Grids::SMIB::ScenarioConfig3 GridParams;

// Generator parameters
const Examples::Components::SynchronousGeneratorKundur::MachineParameters
    syngenKundur;

// Excitation system
const Examples::Components::ExcitationSystemEremia::Parameters excitationEremia;

// Turbine Goverour
const Examples::Components::TurbineGovernor::TurbineGovernorPSAT1
    turbineGovernor;

int main(int argc, char *argv[]) {

  //Simultion parameters
  Real switchClosed = GridParams.SwitchClosed;
  Real switchOpen = GridParams.SwitchOpen;
  Real startTimeFault = 1.0;
  Real endTimeFault = 1.1;
  Real finalTime = 5;
  Real timeStep = 1e-3;
  Real H = syngenKundur.H;
  bool withExciter = false;
  bool withTurbineGovernor = false;
  std::string SGModel = "4";
  std::string stepSize_str = "";
  std::string inertia_str = "";

  // Command line args processing
  CommandLineArgs args(argc, argv);
  if (argc > 1) {
    if (args.options.find("SGModel") != args.options.end()) {
      SGModel = args.getOptionString("SGModel");
      if (args.options.find("WITHEXCITER") != args.options.end())
        withExciter = args.getOptionBool("WITHEXCITER");
      if (args.options.find("WithTurbineGovernor") != args.options.end())
        withTurbineGovernor = args.getOptionBool("WithTurbineGovernor");
    }
    if (args.options.find("Inertia") != args.options.end()) {
      H = args.getOptionReal("Inertia");
      inertia_str = "_Inertia_" + std::to_string(H);
    }
    if (args.options.find("StepSize") != args.options.end()) {
      timeStep = args.getOptionReal("StepSize");
      stepSize_str = "_StepSize_" + std::to_string(timeStep);
    }
  }

  Real logDownSampling;
  if (timeStep < 100e-6)
    logDownSampling = floor(100e-6 / timeStep);
  else
    logDownSampling = 1.0;
  Logger::Level logLevel = Logger::Level::off;
  std::string simName = "DP_SynGen" + SGModel + "Order_VBR_Load_Fault" +
                        stepSize_str + inertia_str;

  // ----- Dynamic simulation ------
  String simNameDP = simName;
  Logger::setLogDir("logs/" + simNameDP);

  // Nodes
  auto initVoltN1 = std::vector<Complex>(
      {Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle),
               GridParams.VnomMV * sin(GridParams.initVoltAngle))});
  auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initVoltN1);

  // Synchronous generator
  auto genDP = GeneratorFactory::createGenDP(SGModel, "SynGen", logLevel);
  genDP->setOperationalParametersPerUnit(
      syngenKundur.nomPower, syngenKundur.nomVoltage, syngenKundur.nomFreq, H,
      syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, syngenKundur.Ld_t,
      syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
      syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s,
      syngenKundur.Tq0_s);
  genDP->setInitialValues(
      GridParams.initComplexElectricalPower, GridParams.mechPower,
      Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle),
              GridParams.VnomMV * sin(GridParams.initVoltAngle)));
  genDP->setModelAsNortonSource(true);

  std::shared_ptr<CPS::Signal::ExciterDC1Simp> exciterDP = nullptr;
  if (withExciter) {
    auto p = std::make_shared<CPS::Signal::ExciterDC1SimpParameters>();
    p->Ta = excitationEremia.Ta;
    p->Ka = excitationEremia.Ka;
    p->Tef = excitationEremia.Te; // Te → Tef
    p->Kef = excitationEremia.Ke; // Ke → Kef
    p->Tf = excitationEremia.Tf;
    p->Kf = excitationEremia.Kf;
    p->Tr = excitationEremia.Tr;

    p->Aef = 0.0;
    p->Bef = 0.0;
    p->MaxVa = 1.0; // old limits were +1.0 / -0.9
    p->MinVa = -0.9;

    exciterDP = std::make_shared<CPS::Signal::ExciterDC1Simp>("SynGen_Exciter",
                                                              logLevel);
    exciterDP->setParameters(p);

    genDP->addExciter(exciterDP);
  }

  // Turbine Governor
  std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernorDP = nullptr;
  if (withTurbineGovernor) {
    turbineGovernorDP =
        Signal::TurbineGovernorType1::make("SynGen_TurbineGovernor", logLevel);
    turbineGovernorDP->setParameters(
        turbineGovernor.T3, turbineGovernor.T4, turbineGovernor.T5,
        turbineGovernor.Tc, turbineGovernor.Ts, turbineGovernor.R,
        turbineGovernor.Tmin, turbineGovernor.Tmax, turbineGovernor.OmegaRef);
    genDP->addGovernor(turbineGovernorDP);
  }

  // Load
  auto load = CPS::DP::Ph1::RXLoad::make("Load", logLevel);
  load->setParameters(GridParams.initActivePower, GridParams.initReactivePower,
                      GridParams.VnomMV);

  //Breaker
  auto fault = CPS::DP::Ph1::Switch::make("Br_fault", logLevel);
  fault->setParameters(switchOpen, switchClosed);
  fault->open();

  // Topology
  genDP->connect({n1DP});
  load->connect({n1DP});
  fault->connect({SP::SimNode::GND, n1DP});
  auto systemDP = SystemTopology(GridParams.nomFreq, SystemNodeList{n1DP},
                                 SystemComponentList{genDP, load, fault});

  // Logging
  auto loggerDP = DataLogger::make(simNameDP, true, logDownSampling);
  loggerDP->logAttribute("v_gen", genDP->attribute("v_intf"));
  loggerDP->logAttribute("i_gen", genDP->attribute("i_intf"));
  loggerDP->logAttribute("Te", genDP->attribute("Te"));
  loggerDP->logAttribute("delta", genDP->attribute("delta"));
  loggerDP->logAttribute("w_r", genDP->attribute("w_r"));
  loggerDP->logAttribute("Vdq0", genDP->attribute("Vdq0"));
  loggerDP->logAttribute("Idq0", genDP->attribute("Idq0"));

  // Exciter
  if (withExciter) {
    loggerDP->logAttribute("Ef", exciterDP->attribute("Ef"));
  }

  // Turbine Governor
  if (withTurbineGovernor) {
    loggerDP->logAttribute("Tm", turbineGovernorDP->attribute("Tm"));
  }

  Simulation simDP(simNameDP, logLevel);
  simDP.doInitFromNodesAndTerminals(true);
  simDP.setSystem(systemDP);
  simDP.setTimeStep(timeStep);
  simDP.setFinalTime(finalTime);
  simDP.setDomain(Domain::DP);
  simDP.addLogger(loggerDP);
  simDP.doSystemMatrixRecomputation(true);

  // Events
  auto sw1 = SwitchEvent::make(startTimeFault, fault, true);
  simDP.addEvent(sw1);

  auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
  simDP.addEvent(sw2);

  simDP.run();
}
