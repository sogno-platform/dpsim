/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <CIMModel.hpp>
#include <IEC61970.hpp>
#include <CIMExceptions.hpp>
#include <memory>

#define READER_CPP
#include <cps/CIM/Reader.h>

using namespace CPS;
using namespace CPS::CIM;
using CIMPP::UnitMultiplier;

namespace fs = std::experimental::filesystem;

Reader::Reader(String name, Logger::Level logLevel, Logger::Level componentLogLevel) {
	mSLog = Logger::get(name + "_CIM", logLevel);

	mModel = new CIMModel();
	mModel->setDependencyCheckOff();
	mComponentLogLevel = componentLogLevel;
}

Reader::~Reader() {
	delete mModel;
}

Real Reader::unitValue(Real value, CIMPP::UnitMultiplier mult) {
	switch (mult) {
	case UnitMultiplier::p:
		value *= 1e-12;
		break;
	case UnitMultiplier::n:
		value *= 1e-9;
		break;
	case UnitMultiplier::micro:
		value *= 1e-6;
		break;
	case UnitMultiplier::m:
		value *= 1e-3;
		break;
	case UnitMultiplier::c:
		value *= 1e-2;
		break;
	case UnitMultiplier::d:
		value *= 1e-1;
		break;
	case UnitMultiplier::k:
		value *= 1e3;
		break;
	case UnitMultiplier::M:
		value *= 1e6;
		break;
	case UnitMultiplier::G:
		value *= 1e9;
		break;
	case UnitMultiplier::T:
		value *= 1e12;
		break;
	default:
		break;
	}
	return value;
}

TopologicalPowerComp::Ptr Reader::mapComponent(BaseClass* obj) {
	if (CIMPP::ACLineSegment *line = dynamic_cast<CIMPP::ACLineSegment*>(obj))
		return mapACLineSegment(line);
	if (CIMPP::EnergyConsumer *consumer = dynamic_cast<CIMPP::EnergyConsumer*>(obj))
		return mapEnergyConsumer(consumer);
	if (CIMPP::PowerTransformer *trans = dynamic_cast<CIMPP::PowerTransformer*>(obj))
		return mapPowerTransformer(trans);
	if (CIMPP::SynchronousMachine *syncMachine = dynamic_cast<CIMPP::SynchronousMachine*>(obj))
		return mapSynchronousMachine(syncMachine);
	if (CIMPP::ExternalNetworkInjection *extnet = dynamic_cast<CIMPP::ExternalNetworkInjection*>(obj))
		return mapExternalNetworkInjection(extnet);
	if (CIMPP::EquivalentShunt *shunt = dynamic_cast<CIMPP::EquivalentShunt*>(obj))
		return mapEquivalentShunt(shunt);
	return nullptr;
}

void Reader::addFiles(const fs::path &filename) {
	if (!mModel->addCIMFile(filename.string()))
		mSLog->error("Failed to read file {}", filename);
}

void Reader::addFiles(const std::list<fs::path> &filenames) {
	for (auto filename : filenames)
		addFiles(filename);
}

void Reader::parseFiles() {
	try {
		mModel->parseFiles();
	}
	catch (...) {
		mSLog->error("Failed to parse CIM files");
		return;
	}

	mSLog->info("#### List of TopologicalNodes, associated Terminals and Equipment");
	for (auto obj : mModel->Objects) {
		if (CIMPP::TopologicalNode* topNode = dynamic_cast<CIMPP::TopologicalNode*>(obj)) {
			if (mDomain == Domain::EMT)
				processTopologicalNode<Real>(topNode);
			else
				processTopologicalNode<Complex>(topNode);
		}
	}

	// Collect voltage state variables associated to nodes that are used
	// for various components.
	mSLog->info("#### List of Node voltages and Terminal power flow data");
	for (auto obj : mModel->Objects) {
		// Check if object is of class SvVoltage
		if (CIMPP::SvVoltage* volt = dynamic_cast<CIMPP::SvVoltage*>(obj)) {
			processSvVoltage(volt);
		}
		// Check if object is of class SvPowerFlow
		else if (CIMPP::SvPowerFlow* flow = dynamic_cast<CIMPP::SvPowerFlow*>(obj)) {
			processSvPowerFlow(flow);
		}
	}

	mSLog->info("#### Create other components");
	for (auto obj : mModel->Objects) {

		// Check if object is not TopologicalNode, SvVoltage or SvPowerFlow
		if (!dynamic_cast<CIMPP::TopologicalNode*>(obj)
			&& !dynamic_cast<CIMPP::SvVoltage*>(obj)
			&& !dynamic_cast<CIMPP::SvPowerFlow*>(obj)) {

			if (CIMPP::IdentifiedObject* idObj = dynamic_cast<CIMPP::IdentifiedObject*>(obj)) {

				// Check if object is already in equipment list
				if (mPowerflowEquipment.find(idObj->mRID) == mPowerflowEquipment.end()) {
					TopologicalPowerComp::Ptr comp = mapComponent(obj);
					if (comp)
						mPowerflowEquipment.insert(std::make_pair(comp->uid(), comp));
				}
			}
		}
	}

	mSLog->info("#### Check topology for unconnected components");
	for (auto pfe : mPowerflowEquipment) {
		auto c = pfe.second;

		if (mDomain == Domain::EMT) {
			if(SimPowerComp<Real>::Ptr powercomp = std::dynamic_pointer_cast<SimPowerComp<Real>>(c)) {
				if (powercomp->terminalNumberConnected() < powercomp->terminalNumber())
					throw InvalidTopology();
			}
		}
		else {
			if(SimPowerComp<Complex>::Ptr powercomp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(c)) {
				if (powercomp->terminalNumberConnected() < powercomp->terminalNumber())
					throw InvalidTopology();
			}
		}
	}
}

SystemTopology Reader::loadCIM(Real systemFrequency, const fs::path &filename, Domain domain, PhaseType phase, GeneratorType genType) {
	mFrequency = systemFrequency;
	mOmega = 2 * PI*mFrequency;
	mDomain = domain;
	mPhase = phase;
	mGeneratorType = genType;
	addFiles(filename);
	parseFiles();
	return systemTopology();
}

SystemTopology Reader::loadCIM(Real systemFrequency, const std::list<fs::path> &filenames, Domain domain, PhaseType phase, GeneratorType genType) {
	mFrequency = systemFrequency;
	mOmega = 2 * PI*mFrequency;
	mDomain = domain;
	mPhase = phase;
	mGeneratorType = genType;
	addFiles(filenames);
	parseFiles();
	return systemTopology();
}

void Reader::processSvVoltage(CIMPP::SvVoltage* volt) {
	CIMPP::TopologicalNode* node = volt->TopologicalNode;
	if (!node) {
		mSLog->warn("SvVoltage references missing Topological Node, ignoring");
		return;
	}
	auto search = mPowerflowNodes.find(node->mRID);
	if (search == mPowerflowNodes.end()) {
		mSLog->warn("SvVoltage references Topological Node {}"
			" missing from mTopNodes, ignoring", node->mRID);
		return;
	}

	Real voltageAbs = Reader::unitValue(volt->v.value, UnitMultiplier::k);

	try{
		mSLog->info("    Angle={}", (float)volt->angle.value);
	}catch(ReadingUninitializedField* e ){
		volt->angle.value = 0;
		std::cerr<< "Uninitialized Angle for SVVoltage at " << volt->TopologicalNode->name << ".Setting default value of " << volt->angle.value << std::endl;
	}
	Real voltagePhase = volt->angle.value * PI / 180;
	mPowerflowNodes[node->mRID]->setInitialVoltage(std::polar<Real>(voltageAbs, voltagePhase));

	mSLog->info("Node {} MatrixNodeIndex {}: {} V, {} deg",
		mPowerflowNodes[node->mRID]->uid(),
		mPowerflowNodes[node->mRID]->matrixNodeIndex(),
		std::abs(mPowerflowNodes[node->mRID]->initialSingleVoltage()),
		std::arg(mPowerflowNodes[node->mRID]->initialSingleVoltage())*180/PI
	);
}

void Reader::processSvPowerFlow(CIMPP::SvPowerFlow* flow) {
	CIMPP::Terminal* term = flow->Terminal;

	mPowerflowTerminals[term->mRID]->setPower(
		Complex(Reader::unitValue(flow->p.value, UnitMultiplier::M),
		Reader::unitValue(flow->q.value, UnitMultiplier::M)));

	mSLog->warn("Terminal {}: {} W + j {} Var",
		term->mRID,
		mPowerflowTerminals[term->mRID]->singleActivePower(),
		mPowerflowTerminals[term->mRID]->singleReactivePower());
}

SystemTopology Reader::systemTopology() {
	SystemTopology system(mFrequency);

	for (auto comp : mPowerflowEquipment) {
		system.addComponent(comp.second);
		// TODO support Real
		if (SimPowerComp<Complex>::Ptr powercomp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp.second)) {
			for (auto term : powercomp->topologicalTerminals()) {
				TopologicalNode::Ptr node=term->topologicalNodes();
			//TopologicalNode::Ptr node = powercomp->topologicalTerminals().back()->topologicalNodes();
			if (system.mComponentsAtNode.find(node) == system.mComponentsAtNode.end()) {
				TopologicalPowerComp::List complist;
				complist.push_back(powercomp);
				system.mComponentsAtNode.insert(std::make_pair(node, complist));
			}
			else {
				system.mComponentsAtNode[node].push_back(powercomp);
			}
			}
		}
	}

	system.mNodes.resize(mPowerflowNodes.size());

	for (auto node : mPowerflowNodes) {
		// The index of the node in the list should not matter anymore
		//system.mNodes[node.second->matrixNodeIndex()] = node.second;
		system.addNodeAt(node.second, node.second->matrixNodeIndex());
	}

	return system;
}

Matrix::Index Reader::mapTopologicalNode(String mrid) {
	auto search = mPowerflowNodes.find(mrid);
	if (search == mPowerflowNodes.end()) {
		return -1;
	}
	return search->second->matrixNodeIndex();
}

TopologicalPowerComp::Ptr Reader::mapEnergyConsumer(CIMPP::EnergyConsumer* consumer) {
	mSLog->info("    Found EnergyConsumer {}", consumer->name);
	if (mDomain == Domain::EMT) {
		if (mPhase == PhaseType::ABC) {
			return std::make_shared<EMT::Ph3::RXLoad>(consumer->mRID, consumer->name, mComponentLogLevel);
		}
		else
		{
		mSLog->info("    RXLoad for EMT not implemented yet");
		return std::make_shared<DP::Ph1::RXLoad>(consumer->mRID, consumer->name, mComponentLogLevel);
		}
	}
	else if (mDomain == Domain::SP) {
		auto load = std::make_shared<SP::Ph1::Load>(consumer->mRID, consumer->name, mComponentLogLevel);

		// TODO: Use EnergyConsumer.P and EnergyConsumer.Q if available, overwrite if existent SvPowerFlow data
		/*
		Real p = 0;
		Real q = 0;
		if (consumer->p.value){
			p = unitValue(consumer->p.value,UnitMultiplier::M);
		}
		if (consumer->q.value){
			q = unitValue(consumer->q.value,UnitMultiplier::M);
		}
		load->setParameters(p, q, 0);
		*/

		// P and Q values will be set according to SvPowerFlow data	
		load->modifyPowerFlowBusType(PowerflowBusType::PQ); // for powerflow solver set as PQ component as default
		return load;
	}
	else {
		if (mUseProtectionSwitches)
			return std::make_shared<DP::Ph1::RXLoadSwitch>(consumer->mRID, consumer->name, mComponentLogLevel);
		else
			return std::make_shared<DP::Ph1::RXLoad>(consumer->mRID, consumer->name, mComponentLogLevel);
	}
}

TopologicalPowerComp::Ptr Reader::mapACLineSegment(CIMPP::ACLineSegment* line) {
	mSLog->info("    Found ACLineSegment {} r={} x={} bch={} gch={}", line->name,
		(float) line->r.value,
		(float) line->x.value,
		(float) line->bch.value,
		(float) line->gch.value);

	Real resistance = line->r.value;
	Real inductance = line->x.value / mOmega;

	// By default there is always a small conductance to ground to
	// avoid problems with floating nodes.
	Real capacitance = mShuntCapacitorValue;
	Real conductance = mShuntConductanceValue;

	if(line->bch.value > 1e-9 && !mSetShuntCapacitor)
		capacitance = Real(line->bch.value / mOmega);

	if(line->gch.value > 1e-9 && !mSetShuntConductance)
		conductance = Real(line->gch.value);

	Real baseVoltage = determineBaseVoltageAssociatedWithEquipment(line);

	if (mDomain == Domain::EMT) {
		if (mPhase == PhaseType::ABC) {
			Matrix res_3ph = CPS::Math::singlePhaseParameterToThreePhase(resistance);
			Matrix ind_3ph = CPS::Math::singlePhaseParameterToThreePhase(inductance);
			Matrix cap_3ph = CPS::Math::singlePhaseParameterToThreePhase(capacitance);
			Matrix cond_3ph = CPS::Math::singlePhaseParameterToThreePhase(conductance);

			auto cpsLine = std::make_shared<EMT::Ph3::PiLine>(line->mRID, line->name, mComponentLogLevel);
			cpsLine->setParameters(res_3ph, ind_3ph, cap_3ph, cond_3ph);
			return cpsLine;
		}
		else {
			mSLog->info("    PiLine for EMT not implemented yet");
			auto cpsLine = std::make_shared<DP::Ph1::PiLine>(line->mRID, line->name, mComponentLogLevel);
			cpsLine->setParameters(resistance, inductance, capacitance, conductance);
			return cpsLine;
		}
	}
	else if (mDomain == Domain::SP) {
		auto cpsLine = std::make_shared<SP::Ph1::PiLine>(line->mRID, line->name, mComponentLogLevel);
		cpsLine->setParameters(resistance, inductance, capacitance, conductance);
		cpsLine->setBaseVoltage(baseVoltage);
		return cpsLine;
	}
	else {
		auto cpsLine = std::make_shared<DP::Ph1::PiLine>(line->mRID, line->name, mComponentLogLevel);
		cpsLine->setParameters(resistance, inductance, capacitance, conductance);
		return cpsLine;
	}

}

TopologicalPowerComp::Ptr Reader::mapPowerTransformer(CIMPP::PowerTransformer* trans) {
	if (trans->PowerTransformerEnd.size() != 2 && trans->PowerTransformerEnd.size() != 3) {
		mSLog->warn("PowerTransformer {} does not have exactly two or three windings, ignoring", trans->name);
		return nullptr;
	}
	mSLog->info("Found PowerTransformer {}", trans->name);

	// assign transformer ends
	CIMPP::PowerTransformerEnd* end1 = nullptr, *end2 = nullptr, *end3 = nullptr;
	for (auto end : trans->PowerTransformerEnd) {
		if (end->Terminal->sequenceNumber == 1) end1 = end;
		else if (end->Terminal->sequenceNumber == 2) end2 = end;
		else if (end->Terminal->sequenceNumber == 3) end3 = end;
		else return nullptr;
	}

	// setting default values for non-set resistances and reactances
	mSLog->info("    PowerTransformerEnd_1 {}", end1->name);
    mSLog->info("    Srated={} Vrated={}", (float) end1->ratedS.value, (float) end1->ratedU.value);
	try {
		mSLog->info("       R={}", (float) end1->r.value);
	} catch(ReadingUninitializedField* e1){
		end1->r.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd1 setting default value of R={}", (float) end1->r.value);
	}
	try {
		mSLog->info("       X={}", (float) end1->x.value);
	} catch(ReadingUninitializedField* e1){
		end1->x.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd1 setting default value of X={}", (float) end1->x.value);
	}
    mSLog->info("    PowerTransformerEnd_2 {}", end2->name);
    mSLog->info("    Srated={} Vrated={}", (float) end2->ratedS.value, (float) end2->ratedU.value);
	try {
		mSLog->info("       R={}", (float) end2->r.value);
	} catch(ReadingUninitializedField* e1){
		end2->r.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd2 setting default value of R={}", (float) end2->r.value);
	}
	try {
		mSLog->info("       X={}", (float) end2->x.value);
	} catch(ReadingUninitializedField* e1){
		end2->x.value = 1e-12;
        mSLog->warn("       Uninitialized value for PowerTrafoEnd2 setting default value of X={}", (float) end2->x.value);
	}

	if (end3) {
		mSLog->info("    PowerTransformerEnd_3 {}", end3->name);
		mSLog->info("    Srated={} Vrated={}", (float) end3->ratedS.value, (float) end3->ratedU.value);
		try {
			mSLog->info("       R={}", (float) end3->r.value);
		} catch(ReadingUninitializedField* e1){
			end3->r.value = 1e-12;
			mSLog->warn("       Uninitialized value for PowerTrafoEnd3 setting default value of R={}", (float) end3->r.value);
		}
		try {
			mSLog->info("       X={}", (float) end3->x.value);
		} catch(ReadingUninitializedField* e1){
			end3->x.value = 1e-12;
			mSLog->warn("       Uninitialized value for PowerTrafoEnd3 setting default value of X={}", (float) end3->x.value);
		}
	}

	if (!end3 && end1->ratedS.value != end2->ratedS.value) {
		mSLog->warn("    PowerTransformerEnds of {} come with distinct rated power values. Using rated power of PowerTransformerEnd_1.", trans->name);
	}

	Real voltageNode1 = unitValue(end1->ratedU.value, UnitMultiplier::k);
	Real voltageNode2 = unitValue(end2->ratedU.value, UnitMultiplier::k);
	
	Real ratedPower1 = unitValue(end1->ratedS.value, UnitMultiplier::M);
	Real ratedPower2 = unitValue(end2->ratedS.value, UnitMultiplier::M);

	Real ratioAbsNominal = voltageNode1 / voltageNode2;
	Real ratioAbs = ratioAbsNominal;

	// use normalStep from RatioTapChanger
	if (end1->RatioTapChanger && !end3) {
		ratioAbs = voltageNode1 / voltageNode2 * (1 + (end1->RatioTapChanger->normalStep - end1->RatioTapChanger->neutralStep) * end1->RatioTapChanger->stepVoltageIncrement.value / 100);
	}

	// if corresponding SvTapStep available, use instead tap position from there
	if (end1->RatioTapChanger && !end3) {
		for (auto obj : mModel->Objects) {
			auto tapStep = dynamic_cast<CIMPP::SvTapStep*>(obj);
			if (tapStep && tapStep->TapChanger == end1->RatioTapChanger) {
				ratioAbsNominal = voltageNode1 / voltageNode2 * (1 + (tapStep->position - end1->RatioTapChanger->neutralStep) * end1->RatioTapChanger->stepVoltageIncrement.value / 100);
			}
		}
	}

	if (!end3) {
		// TODO: To be extracted from cim class
		Real ratioPhase = 0;

		// Calculate resistance and inductance referred to high voltage side
		Real resistance = 0;
		Real inductance = 0;

		if (voltageNode1 >= voltageNode2 && abs(end1->x.value) > 1e-12) {
			inductance = end1->x.value / mOmega;
			resistance = end1->r.value;
		} else if (voltageNode1 >= voltageNode2 && abs(end2->x.value) > 1e-12) {
			inductance = end2->x.value / mOmega * std::pow(ratioAbsNominal, 2);
			resistance = end2->r.value * std::pow(ratioAbsNominal, 2);
		} else if (voltageNode2 > voltageNode1 && abs(end2->x.value) > 1e-12) {
			inductance = end2->x.value / mOmega;
			resistance = end2->r.value;
		} else if (voltageNode2 > voltageNode1 && abs(end1->x.value) > 1e-12) {
			inductance = end1->x.value / mOmega / std::pow(ratioAbsNominal, 2);
			resistance = end1->r.value / std::pow(ratioAbsNominal, 2);
		}

		if (mDomain == Domain::EMT) {
			if (mPhase == PhaseType::ABC) {
				Matrix resistance_3ph = CPS::Math::singlePhaseParameterToThreePhase(resistance);
				Matrix inductance_3ph = CPS::Math::singlePhaseParameterToThreePhase(inductance);
				Bool withResistiveLosses = resistance > 0;
				auto transformer = std::make_shared<EMT::Ph3::Transformer>(trans->mRID, trans->name, mComponentLogLevel, withResistiveLosses);
				transformer->setParameters(voltageNode1, voltageNode2, ratioAbs, ratioPhase, resistance_3ph, inductance_3ph);
				return transformer;
			} else {
				mSLog->info("    Transformer for EMT not implemented yet");
				return nullptr;
			}
		} else if (mDomain == Domain::SP) {
			auto transformer = std::make_shared<SP::Ph1::Transformer>(trans->mRID, trans->name, mComponentLogLevel);
			transformer->setParameters(voltageNode1, voltageNode2, ratedPower1, ratioAbs, ratioPhase, resistance, inductance);
			Real baseVolt = voltageNode1 >= voltageNode2 ? voltageNode1 : voltageNode2;
			transformer->setBaseVoltage(baseVolt);
			return transformer;
		} else {
			Bool withResistiveLosses = resistance > 0;
			auto transformer = std::make_shared<DP::Ph1::Transformer>(trans->mRID, trans->name, mComponentLogLevel, withResistiveLosses);
			transformer->setParameters(voltageNode1, voltageNode2, ratioAbs, ratioPhase, resistance, inductance);
			return transformer;
		}
	}


	else {
		Real ratedPower3 = unitValue(end3->ratedS.value, UnitMultiplier::M);
		Real voltageNode3 = unitValue(end2->ratedU.value, UnitMultiplier::k);

		Real ratioAbs1 = 1.0;
		// use normalStep from RatioTapChanger
		if (end1->RatioTapChanger) {
			ratioAbs1 = (1 + (end1->RatioTapChanger->normalStep - end1->RatioTapChanger->neutralStep) * end1->RatioTapChanger->stepVoltageIncrement.value / 100);
		}
		// if corresponding SvTapStep available, use instead tap position from there
		if (end1->RatioTapChanger) {
			for (auto obj : mModel->Objects) {
				auto tapStep = dynamic_cast<CIMPP::SvTapStep*>(obj);
				if (tapStep && tapStep->TapChanger == end1->RatioTapChanger) {
					ratioAbs1 = (1 + (tapStep->position - end1->RatioTapChanger->neutralStep) * end1->RatioTapChanger->stepVoltageIncrement.value / 100);
				}
			}
		}

		Real ratioAbs2 = 1.0;
		// use normalStep from RatioTapChanger
		if (end2->RatioTapChanger) {
			ratioAbs2 = (1 + (end2->RatioTapChanger->normalStep - end2->RatioTapChanger->neutralStep) * end2->RatioTapChanger->stepVoltageIncrement.value / 100);
		}
		// if corresponding SvTapStep available, use instead tap position from there
		if (end2->RatioTapChanger) {
			for (auto obj : mModel->Objects) {
				auto tapStep = dynamic_cast<CIMPP::SvTapStep*>(obj);
				if (tapStep && tapStep->TapChanger == end2->RatioTapChanger) {
					ratioAbs2 = (1 + (tapStep->position - end2->RatioTapChanger->neutralStep) * end2->RatioTapChanger->stepVoltageIncrement.value / 100);
				}
			}
		}

		Real ratioAbs3 = 1.0;
		// use normalStep from RatioTapChanger
		if (end3->RatioTapChanger) {
			ratioAbs3 = (1 + (end3->RatioTapChanger->normalStep - end3->RatioTapChanger->neutralStep) * end3->RatioTapChanger->stepVoltageIncrement.value / 100);
		}
		// if corresponding SvTapStep available, use instead tap position from there
		if (end3->RatioTapChanger) {
			for (auto obj : mModel->Objects) {
				auto tapStep = dynamic_cast<CIMPP::SvTapStep*>(obj);
				if (tapStep && tapStep->TapChanger == end3->RatioTapChanger) {
					ratioAbs3 = (1 + (tapStep->position - end3->RatioTapChanger->neutralStep) * end3->RatioTapChanger->stepVoltageIncrement.value / 100);
				}
			}
		}

		// TODO: To be extracted from cim class
		Real ratioPhase1 = 0;
		Real ratioPhase2 = 0;
		Real ratioPhase3 = 0;

		// Calculate resistance and inductance referred to high voltage side
		Real resistance1 = 0;
		Real resistance2 = 0;
		Real resistance3 = 0;
		Real inductance1 = 0;
		Real inductance2 = 0;
		Real inductance3 = 0;

		// TODO: Should verify if any inductance is < 1e-12
		resistance1 = end1->r.value;
		resistance2 = end2->r.value;
		resistance3 = end3->r.value;
		inductance1 = end1->x.value / mOmega;
		inductance2 = end2->x.value / mOmega;
		inductance3 = end1->x.value / mOmega;

		if (mDomain == Domain::EMT) {
			mSLog->info("    3-Winding transformer for EMT not implemented yet");
			return nullptr;
		} else if (mDomain == Domain::SP) {
			auto transformer = std::make_shared<SP::Ph1::Transformer3W>(trans->mRID, trans->name, mComponentLogLevel);
			transformer->setParameters(
				voltageNode1, voltageNode2, voltageNode3, 
				ratedPower1, ratedPower2, ratedPower3, 
				ratioAbs1, ratioAbs2, ratioAbs3, 
				ratioPhase1, ratioPhase2, ratioPhase3,
				resistance1, resistance2, resistance3, 
				inductance1, inductance2, inductance3
			);
			Real baseVolt = voltageNode1 >= voltageNode2 ? voltageNode1 : voltageNode2 >= voltageNode3 ? voltageNode2 : voltageNode3;
			transformer->setBaseVoltage(baseVolt);
			return transformer;
		} else {
			mSLog->info("    3-Winding transformer for DP not implemented yet");
			return nullptr;
		}

	}
}

TopologicalPowerComp::Ptr Reader::mapSynchronousMachine(CIMPP::SynchronousMachine* machine) {
	mSLog->info("    Found  Synchronous machine {}", machine->name);

	if (mDomain == Domain::DP) {
		if (mGeneratorType == GeneratorType::TransientStability) {
			mSLog->info("    GeneratorType is TransientStability.");
			Real directTransientReactance;
			Real inertiaCoefficient;
			Real ratedPower;
			Real ratedVoltage;
			for (auto obj : mModel->Objects) {
				// Check if object is not TopologicalNode, SvVoltage or SvPowerFlow
				if (CIMPP::SynchronousMachineTimeConstantReactance* genDyn =
					dynamic_cast<CIMPP::SynchronousMachineTimeConstantReactance*>(obj)) {
					if (genDyn->SynchronousMachine->mRID == machine->mRID) {
						directTransientReactance = genDyn->xDirectTrans.value;
						inertiaCoefficient = genDyn->inertia.value;

						ratedPower = unitValue(machine->ratedS.value, UnitMultiplier::M);

						ratedVoltage = unitValue(machine->ratedU.value, UnitMultiplier::k);
						auto gen = DP::Ph1::SynchronGeneratorTrStab::make(machine->mRID, machine->name, mComponentLogLevel);
						gen->setStandardParametersPU(ratedPower, ratedVoltage, mFrequency,
						directTransientReactance, inertiaCoefficient);
						return gen;
					}
				}
			}
			mSLog->warn("    Dynamic parameters for {} not found. Not able to instantiate SynchronGeneratorTrStab.", machine->name);
		} else if (mGeneratorType == GeneratorType::IdealVoltageSource) {
			mSLog->info("    GeneratorType is IdealVoltageSource.");
			return std::make_shared<DP::Ph1::SynchronGeneratorIdeal>(machine->mRID, machine->name, mComponentLogLevel);
		} else if (mGeneratorType == GeneratorType::None) {
			throw SystemError("GeneratorType is None. Specify!");
		} else {
			throw SystemError("GeneratorType setting unfeasible.");
		}
	} else if (mDomain == Domain::SP) {
		if (mGeneratorType == GeneratorType::TransientStability) {
			mSLog->info("    GeneratorType is TransientStability.");
			Real directTransientReactance;
			Real inertiaCoefficient;
			Real ratedPower;
			Real ratedVoltage;

			for (auto obj : mModel->Objects) {
				// Check if object is not TopologicalNode, SvVoltage or SvPowerFlow
				if (CIMPP::SynchronousMachineTimeConstantReactance* genDyn =
					dynamic_cast<CIMPP::SynchronousMachineTimeConstantReactance*>(obj)) {
					if (genDyn->SynchronousMachine->mRID == machine->mRID) {
						directTransientReactance = genDyn->xDirectTrans.value;
						inertiaCoefficient = genDyn->inertia.value;

						ratedPower = unitValue(machine->ratedS.value, UnitMultiplier::M);

						ratedVoltage = unitValue(machine->ratedU.value, UnitMultiplier::k);
						auto gen = SP::Ph1::SynchronGeneratorTrStab::make(machine->mRID, machine->name, mComponentLogLevel);
						gen->setStandardParametersPU(ratedPower, ratedVoltage, mFrequency,
						directTransientReactance, inertiaCoefficient);
						return gen;
					}
				}
			}
		} else if (mGeneratorType == GeneratorType::PVNode) {
			mSLog->info("    GeneratorType is PVNode.");
			for (auto obj : mModel->Objects) {
				if (CIMPP::GeneratingUnit* genUnit = dynamic_cast<CIMPP::GeneratingUnit*>(obj)) {
					for (auto syncGen : genUnit->RotatingMachine) {
						if (syncGen->mRID == machine->mRID){
							// Check whether relevant input data are set, otherwise set default values
							Real setPointActivePower = 0;
							Real setPointVoltage = 0;
							Real maximumReactivePower = 1e12;
							try{
								setPointActivePower = unitValue(genUnit->initialP.value, UnitMultiplier::M);
								mSLog->info("    setPointActivePower={}", setPointActivePower);
							}catch(ReadingUninitializedField* e){
								std::cerr << "Uninitalized setPointActivePower for GeneratingUnit " << machine->name << ". Using default value of " << setPointActivePower << std::endl;
							}
							if (machine->RegulatingControl) {
								setPointVoltage = unitValue(machine->RegulatingControl->targetValue.value, UnitMultiplier::k);
								mSLog->info("    setPointVoltage={}", setPointVoltage);
							} else {
								std::cerr << "Uninitalized setPointVoltage for GeneratingUnit " <<  machine->name << ". Using default value of " << setPointVoltage << std::endl;
							}
							try{
								maximumReactivePower = unitValue(machine->maxQ.value, UnitMultiplier::M);
								mSLog->info("    maximumReactivePower={}", maximumReactivePower);
							}catch(ReadingUninitializedField* e){
								std::cerr << "Uninitalized maximumReactivePower for GeneratingUnit " <<  machine->name << ". Using default value of " << maximumReactivePower << std::endl;
							}

							auto gen = std::make_shared<SP::Ph1::SynchronGenerator>(machine->mRID, machine->name, mComponentLogLevel);
								gen->setParameters(unitValue(machine->ratedS.value, UnitMultiplier::M),
										unitValue(machine->ratedU.value, UnitMultiplier::k),
										setPointActivePower,
										setPointVoltage,
										PowerflowBusType::PV);
								gen->setBaseVoltage(unitValue(machine->ratedU.value, UnitMultiplier::k));
							return gen;
						}
					}
				}
			}
			mSLog->info("no corresponding initial power for {}", machine->name);
			return std::make_shared<SP::Ph1::SynchronGenerator>(machine->mRID, machine->name, mComponentLogLevel);
		} else if (mGeneratorType == GeneratorType::None) {
			throw SystemError("GeneratorType is None. Specify!");
		} else {
			throw SystemError("GeneratorType setting unfeasible.");
		}
	} else {
        throw SystemError("Mapping of SynchronousMachine for EMT not existent yet!");
    }
	return nullptr;
}

TopologicalPowerComp::Ptr Reader::mapExternalNetworkInjection(CIMPP::ExternalNetworkInjection* extnet) {
	mSLog->info("Found External Network Injection {}", extnet->name);

	Real baseVoltage = determineBaseVoltageAssociatedWithEquipment(extnet);

	if (mDomain == Domain::EMT) {
		if (mPhase == PhaseType::ABC) {
			return std::make_shared<EMT::Ph3::NetworkInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
		}
		else {
			throw SystemError("Mapping of ExternalNetworkInjection for EMT::Ph1 not existent!");
			return nullptr;
		}
	} else if(mDomain == Domain::SP) {
		if (mPhase == PhaseType::Single) {
			auto cpsextnet = std::make_shared<SP::Ph1::NetworkInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
			cpsextnet->modifyPowerFlowBusType(PowerflowBusType::VD); // for powerflow solver set as VD component as default
			cpsextnet->setBaseVoltage(baseVoltage);

			try {
				if(extnet->RegulatingControl){
					mSLog->info("       Voltage set-point={}", (float) extnet->RegulatingControl->targetValue);
					cpsextnet->setParameters(extnet->RegulatingControl->targetValue*baseVoltage); // assumes that value is specified in CIM data in per unit
				} else {
					mSLog->info("       No voltage set-point defined. Using 1 per unit.");
					cpsextnet->setParameters(1.*baseVoltage);
				}
			} catch (ReadingUninitializedField* e ) {
				std::cerr << "Ignore incomplete RegulatingControl" << std::endl;
			}

			return cpsextnet;
		}
		else {
			throw SystemError("Mapping of ExternalNetworkInjection for SP::Ph3 not existent!");
			return nullptr;
		}
	} else {
		if (mPhase == PhaseType::Single) {
			return std::make_shared<DP::Ph1::NetworkInjection>(extnet->mRID, extnet->name, mComponentLogLevel);
		} else {
			throw SystemError("Mapping of ExternalNetworkInjection for DP::Ph3 not existent!");
			return nullptr;
		}
	}
}

TopologicalPowerComp::Ptr Reader::mapEquivalentShunt(CIMPP::EquivalentShunt* shunt){
	mSLog->info("Found shunt {}", shunt->name);

	Real baseVoltage = determineBaseVoltageAssociatedWithEquipment(shunt);

	auto cpsShunt = std::make_shared<SP::Ph1::Shunt>(shunt->mRID, shunt->name, mComponentLogLevel);
	cpsShunt->setParameters(shunt->g.value, shunt->b.value);
	cpsShunt->setBaseVoltage(baseVoltage);
	return cpsShunt;
}

void Reader::initDynamicSystemTopologyWithPowerflow(SystemTopology& systemPF, SystemTopology& system) {
	for (auto nodePF : systemPF.mNodes) {
		if (auto node = system.node<TopologicalNode>(nodePF->name())) {
			mSLog->info("Updating initial voltage of {} according to powerflow", node->name());
			mSLog->info("Former initial voltage: {}", node->initialSingleVoltage());
			node->setInitialVoltage(std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(nodePF)->singleVoltage());
			mSLog->info("Updated initial voltage: {}", node->initialSingleVoltage());
		}
	}
}

Real Reader::determineBaseVoltageAssociatedWithEquipment(CIMPP::ConductingEquipment* equipment){
	Real baseVoltage = 0;

    // first look for baseVolt object to determine baseVoltage
    for (auto obj : mModel->Objects) {
        if (CIMPP::BaseVoltage* baseVolt = dynamic_cast<CIMPP::BaseVoltage*>(obj)) {
            for (auto comp : baseVolt->ConductingEquipment) {
                if (comp->name == equipment->name) {
                    baseVoltage = unitValue(baseVolt->nominalVoltage.value,UnitMultiplier::k);
                }
            }
        }
    }
    // as second option take baseVoltage of topologicalNode where equipment is connected to
    if(baseVoltage == 0){
        for (auto obj : mModel->Objects) {
			if (CIMPP::TopologicalNode* topNode = dynamic_cast<CIMPP::TopologicalNode*>(obj)) {
                for (auto term : topNode->Terminal) {
					if (term->ConductingEquipment->name == equipment->name) {
                        baseVoltage = unitValue(topNode->BaseVoltage->nominalVoltage.value,UnitMultiplier::k);
					}
				}
			}
		}
    }

	return baseVoltage;
}



template<typename VarType>
void Reader::processTopologicalNode(CIMPP::TopologicalNode* topNode) {
	// Add this node to global node list and assign simulation node incrementally.
	int matrixNodeIndex = Int(mPowerflowNodes.size());
	mPowerflowNodes[topNode->mRID] = SimNode<VarType>::make(topNode->mRID, topNode->name, matrixNodeIndex, mPhase);

	if (mPhase == PhaseType::ABC) {
		mSLog->info("TopologicalNode {} phase A as simulation node {} ", topNode->mRID, mPowerflowNodes[topNode->mRID]->matrixNodeIndex(PhaseType::A));
		mSLog->info("TopologicalNode {} phase B as simulation node {}", topNode->mRID, mPowerflowNodes[topNode->mRID]->matrixNodeIndex(PhaseType::B));
		mSLog->info("TopologicalNode {} phase C as simulation node {}", topNode->mRID, mPowerflowNodes[topNode->mRID]->matrixNodeIndex(PhaseType::C));
	}
	else
		mSLog->info("TopologicalNode id: {}, name: {} as simulation node {}", topNode->mRID, topNode->name, mPowerflowNodes[topNode->mRID]->matrixNodeIndex());

	for (auto term : topNode->Terminal) {
		// Insert Terminal if it does not exist in the map and add reference to node.
		// This could be optimized because the Terminal is searched twice.
		auto cpsTerm = SimTerminal<VarType>::make(term->mRID);
		mPowerflowTerminals.insert(std::make_pair(term->mRID, cpsTerm));
		cpsTerm->setNode(std::dynamic_pointer_cast<SimNode<VarType>>(mPowerflowNodes[topNode->mRID]));

		if (!term->sequenceNumber.initialized)
			term->sequenceNumber = 1;

		mSLog->info("    Terminal {}, sequenceNumber {}", term->mRID, (int) term->sequenceNumber);

		// Try to process Equipment connected to Terminal.
		CIMPP::ConductingEquipment *equipment = term->ConductingEquipment;
		if (!equipment) {
			mSLog->warn("Terminal {} has no Equipment, ignoring!", term->mRID);
		}
		else {
			// Insert Equipment if it does not exist in the map and add reference to Terminal.
			// This could be optimized because the Equipment is searched twice.
			if (mPowerflowEquipment.find(equipment->mRID) == mPowerflowEquipment.end()) {
				TopologicalPowerComp::Ptr comp = mapComponent(equipment);
				if (comp) {
					mPowerflowEquipment.insert(std::make_pair(equipment->mRID, comp));
				} else {
					mSLog->warn("Could not map equipment {}", equipment->mRID);
					continue;
				}
			}

			auto pfEquipment = mPowerflowEquipment.at(equipment->mRID);
			std::dynamic_pointer_cast<SimPowerComp<VarType>>(pfEquipment)->setTerminalAt(
				std::dynamic_pointer_cast<SimTerminal<VarType>>(mPowerflowTerminals[term->mRID]), term->sequenceNumber-1);

			mSLog->info("        Added Terminal {} to Equipment {}", term->mRID, equipment->mRID);
		}
	}
}

template void Reader::processTopologicalNode<Real>(CIMPP::TopologicalNode* topNode);
template void Reader::processTopologicalNode<Complex>(CIMPP::TopologicalNode* topNode);
