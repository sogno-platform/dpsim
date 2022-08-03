#include <DPsim.h>
#include "../Examples.h"


using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

CPS::CIM::Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
CPS::CIM::Examples::Grids::SMIB::ScenarioConfig3 GridParams;


void EMT_Ph3_SG4OVBR_Load()
{
	Real timeStep = 0.0001;
	Real finalTime = 0.1;

	// ----- Dynamic simulation ------
	String simNameEMT = "EMT_Ph3_SG4OVBR_Load"; //"EMT_3ph_SynGen_Load"
	Logger::setLogDir("logs/"+simNameEMT);
	
	// Nodes
	std::vector<Complex> initialVoltage_n1{ GridParams.initTerminalVolt, 
											GridParams.initTerminalVolt * SHIFT_TO_PHASE_B,
											GridParams.initTerminalVolt * SHIFT_TO_PHASE_C
										  };
	auto n1EMT = SimNode<Real>::make("n1EMT", PhaseType::ABC, initialVoltage_n1);

	// Components
	// Synchronous generator
	std::shared_ptr<EMT::Ph3::ReducedOrderSynchronGeneratorVBR> genEMT = nullptr;

	genEMT = EMT::Ph3::SynchronGenerator4OrderVBR::make("SynGen", Logger::Level::info);

	genEMT->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, syngenKundur.H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    genEMT->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower, 
							 GridParams.initTerminalVolt);

	auto load = CPS::EMT::Ph3::RXLoad::make("Load", Logger::Level::info);
	load->setParameters(Math::singlePhaseParameterToThreePhase(GridParams.initActivePower/3), 
						Math::singlePhaseParameterToThreePhase(GridParams.initReactivePower/3),
						GridParams.VnomMV);


	// Topology
	genEMT->connect({ n1EMT });
	load->connect({ n1EMT });

	SystemTopology systemEMT;
	systemEMT = SystemTopology(	GridParams.nomFreq,
								SystemNodeList{n1EMT},
								SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator4OrderVBR>(genEMT), load});
	

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->logAttribute("Etorque", 	genEMT->attribute("Etorque"));

	Simulation simEMT(simNameEMT, Logger::Level::info);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	simEMT.addLogger(loggerEMT);
	simEMT.doSystemMatrixRecomputation(true);
	
	simEMT.run();
	simEMT.logStepTimes(simNameEMT + "_step_times");
}


int main(){
    EMT_Ph3_SG4OVBR_Load();
    return 0;
}