#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

class Example_VS_RSource_PiecewiseLinearInductor {
public:
  Example_VS_RSource_PiecewiseLinearInductor()
      : mTimeStep(0.0001), mFinalTime(0.2), mSourceVoltagePeak(100.0),
        mSourceFrequency(50.0), mSourcePhaseDeg(-90.0), mSourceResistance(0.2),
        mBaseCurrent(1.0) {
    const Real baseFlux =
        std::sqrt(2.0) * mSourceVoltagePeak / (2.0 * PI * mSourceFrequency);

    const std::vector<Real> fluxInPerUnit{0.0, 0.5, 0.9, 1.0};
    const std::vector<Real> currentInPerUnit{0.0, 0.01, 0.1, 10.0};

    for (size_t k = 0; k < fluxInPerUnit.size(); ++k) {
      mFluxBreakpoints.push_back(fluxInPerUnit[k] * baseFlux);
      mCurrentBreakpoints.push_back(currentInPerUnit[k] * mBaseCurrent);
    }
  }

  void runExampleWithoutSteadyStateInitialization() const {
    runExample("EMT_Ph3_PiecewiseLinearInductor", false);
  }

  void runExampleWithSteadyStateInitialization() const {
    runExample("EMT_Ph3_PiecewiseLinearInductor_SteadyStateInit", true);
  }

private:
  void runExample(const String &simName, bool startInSteadyState) const {
    Logger::setLogDir("logs/" + simName);

    // Nodes
    auto n1 = SimNode::make("n1", PhaseType::ABC);
    auto n2 = SimNode::make("n2", PhaseType::ABC);

    if (startInSteadyState) {
      setInitialNodeVoltages(n1, n2);
    }

    // Source
    auto vs = Ph3::VoltageSource::make("VS");
    vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(
                          sourceVoltageRMSLineToLine()),
                      mSourceFrequency);

    // Source resistance
    auto rSource = Ph3::Resistor::make("RSource");
    rSource->setParameters(
        CPS::Math::singlePhaseParameterToThreePhase(mSourceResistance));

    // Piecewise-linear inductor
    auto lPiecewiseLinear = Ph3::PiecewiseLinearInductor::make(
        "LPiecewiseLinear", Logger::Level::debug);
    lPiecewiseLinear->setParameters(mFluxBreakpoints, mCurrentBreakpoints);

    // Topology
    vs->connect(SimNode::List{SimNode::GND, n1});
    rSource->connect(SimNode::List{n1, n2});
    lPiecewiseLinear->connect(SimNode::List{n2, SimNode::GND});

    // System
    auto sys =
        SystemTopology(mSourceFrequency, SystemNodeList{n1, n2},
                       SystemComponentList{vs, rSource, lPiecewiseLinear});

    // Logging
    auto logger = DataLogger::make(simName);
    logger->logAttribute("voltage", n2->attribute("v"));
    logger->logAttribute("current", lPiecewiseLinear->attribute("i_intf"));
    logger->logAttribute("flux", lPiecewiseLinear->attribute("x"));

    // Simulation
    Simulation sim(simName, Logger::Level::info);
    sim.setSystem(sys);
    sim.addLogger(logger);
    sim.setDomain(Domain::EMT);
    sim.setSolverType(Solver::Type::MNA);
    sim.doSystemMatrixRecomputation(true);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.run();
  }

  Complex sourceVoltageRMSLineToLine() const {
    // DPsim source uses cosine convention. A phase angle of -90 deg therefore
    // corresponds to a sine reference in phase A.
    return CPS::Math::polarDeg(PEAK1PH_TO_RMS3PH * mSourceVoltagePeak,
                               mSourcePhaseDeg);
  }

  Real firstSegmentInductance() const {
    const Real deltaFlux = mFluxBreakpoints[1] - mFluxBreakpoints[0];
    const Real deltaCurrent = mCurrentBreakpoints[1] - mCurrentBreakpoints[0];
    return deltaFlux / deltaCurrent;
  }

  void setInitialNodeVoltages(const std::shared_ptr<SimNode> &n1,
                              const std::shared_ptr<SimNode> &n2) const {
    // Manual steady-state initialization:
    // approximate the piecewise-linear inductor by its first linear segment.
    //
    // The stored node voltages use the same convention as the source
    // parametrization: single-phase positive-sequence RMS line-to-line phasors.
    const Complex vSource = sourceVoltageRMSLineToLine();
    const Complex zInductor(0.0, 2.0 * PI * mSourceFrequency *
                                     firstSegmentInductance());
    const Complex zTotal = Complex(mSourceResistance, 0.0) + zInductor;

    const Complex iSteadyState = vSource / zTotal;
    const Complex vInductor = iSteadyState * zInductor;

    n1->setInitialVoltage(vSource);
    n2->setInitialVoltage(vInductor);
  }

private:
  Real mTimeStep;
  Real mFinalTime;

  Real mSourceVoltagePeak;
  Real mSourceFrequency;
  Real mSourcePhaseDeg;
  Real mSourceResistance;

  Real mBaseCurrent;

  std::vector<Real> mFluxBreakpoints;
  std::vector<Real> mCurrentBreakpoints;
};

int main(int argc, char *argv[]) {
  Example_VS_RSource_PiecewiseLinearInductor example;

  example.runExampleWithoutSteadyStateInitialization();
  example.runExampleWithSteadyStateInitialization();

  return 0;
}
