// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

class Example_VS_RCBranch_RLLoad {
public:
  Example_VS_RCBranch_RLLoad()
      : mTimeStep(0.0001), mFinalTime(0.2), mFrequency(50.0),
        mSourceVoltage(CPS::Math::polar(1.0, 0.0)),
        mBranchResistance(CPS::Math::singlePhaseParameterToThreePhase(0.2)),
        mBranchCapacitance(CPS::Math::singlePhaseParameterToThreePhase(500e-6)),
        mLoadResistance(CPS::Math::singlePhaseParameterToThreePhase(1.0)),
        mLoadInductance(CPS::Math::singlePhaseParameterToThreePhase(0.05)) {}

  void runExampleWithMNAComponents() const {
    String simName = "EMT_Ph3_MNA_RCBranch_RLLoad";

    auto n1 = SimNode::make("n1", PhaseType::ABC);
    auto n2 = SimNode::make("n2", PhaseType::ABC);
    auto n3 = SimNode::make("n3", PhaseType::ABC);
    auto n4 = SimNode::make("n4", PhaseType::ABC);

    auto vs = Ph3::VoltageSource::make("VS");
    vs->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mSourceVoltage), mFrequency);

    auto rBranch = Ph3::Resistor::make("RBranch");
    rBranch->setParameters(mBranchResistance);

    auto cBranch = Ph3::Capacitor::make("CBranch");
    cBranch->setParameters(mBranchCapacitance);

    auto rLoad = Ph3::Resistor::make("RLoad");
    rLoad->setParameters(mLoadResistance);

    auto lLoad = Ph3::Inductor::make("LLoad");
    lLoad->setParameters(mLoadInductance);

    vs->connect(SimNode::List{SimNode::GND, n1});
    rBranch->connect(SimNode::List{n1, n2});
    cBranch->connect(SimNode::List{n2, n3});
    rLoad->connect(SimNode::List{n3, n4});
    lLoad->connect(SimNode::List{n4, SimNode::GND});

    auto sys =
        SystemTopology(mFrequency, SystemNodeList{n1, n2, n3, n4},
                       SystemComponentList{vs, rBranch, cBranch, rLoad, lLoad});

    Logger::setLogDir("logs/" + simName);
    auto logger = DataLogger::make(simName);
    logger->logAttribute("v3", n3->attribute("v"));
    logger->logAttribute("iBranch", rBranch->attribute("i_intf"));
    logger->logAttribute("iLoad", rLoad->attribute("i_intf"));

    Simulation sim(simName, Logger::Level::info);
    sim.setSystem(sys);
    sim.addLogger(logger);
    sim.setDomain(Domain::EMT);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.run();
  }

  void runExampleWithRCBranchITypeSSN() const {
    String simName = "EMT_Ph3_ITypeSSN_RCBranch_RLLoad";

    auto n1 = SimNode::make("n1", PhaseType::ABC);
    auto n3 = SimNode::make("n3", PhaseType::ABC);
    auto n4 = SimNode::make("n4", PhaseType::ABC);

    Matrix aMatrix;
    Matrix bMatrix;
    Matrix cMatrix;
    Matrix dMatrix;
    createStateSpaceMatricesForRCBranch(aMatrix, bMatrix, cMatrix, dMatrix);

    auto vs = Ph3::VoltageSource::make("VS");
    vs->setParameters(
        CPS::Math::singlePhaseVariableToThreePhase(mSourceVoltage), mFrequency);

    auto rcBranch =
        Ph3::GenericTwoTerminalITypeSSN::make("RCBranch", Logger::Level::debug);
    rcBranch->setParameters(aMatrix, bMatrix, cMatrix, dMatrix);

    auto rLoad = Ph3::Resistor::make("RLoad");
    rLoad->setParameters(mLoadResistance);

    auto lLoad = Ph3::Inductor::make("LLoad");
    lLoad->setParameters(mLoadInductance);

    vs->connect(SimNode::List{SimNode::GND, n1});
    rcBranch->connect(SimNode::List{n1, n3});
    rLoad->connect(SimNode::List{n3, n4});
    lLoad->connect(SimNode::List{n4, SimNode::GND});

    auto sys = SystemTopology(mFrequency, SystemNodeList{n1, n3, n4},
                              SystemComponentList{vs, rcBranch, rLoad, lLoad});

    Logger::setLogDir("logs/" + simName);
    auto logger = DataLogger::make(simName);
    logger->logAttribute("v3", n3->attribute("v"));
    logger->logAttribute("iBranch", rcBranch->attribute("i_intf"));
    logger->logAttribute("vBranch", rcBranch->attribute("v_intf"));
    logger->logAttribute("iLoad", rLoad->attribute("i_intf"));

    Simulation sim(simName, Logger::Level::info);
    sim.setSystem(sys);
    sim.addLogger(logger);
    sim.setDomain(Domain::EMT);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.run();
  }

private:
  void createStateSpaceMatricesForRCBranch(Matrix &aMatrix, Matrix &bMatrix,
                                           Matrix &cMatrix,
                                           Matrix &dMatrix) const {
    aMatrix = Matrix::Zero(3, 3);
    bMatrix = mBranchCapacitance.inverse();

    cMatrix = Matrix::Identity(3, 3);
    dMatrix = mBranchResistance;
  }

private:
  Real mTimeStep;
  Real mFinalTime;
  Real mFrequency;
  Complex mSourceVoltage;

  Matrix mBranchResistance;
  Matrix mBranchCapacitance;

  Matrix mLoadResistance;
  Matrix mLoadInductance;
};

int main(int argc, char *argv[]) {
  Example_VS_RCBranch_RLLoad example;

  example.runExampleWithMNAComponents();
  example.runExampleWithRCBranchITypeSSN();

  return 0;
}
