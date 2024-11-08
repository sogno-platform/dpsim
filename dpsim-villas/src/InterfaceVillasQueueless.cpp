/* Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2023-2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-villas/InterfaceVillasQueueless.h>
#include <dpsim-villas/InterfaceWorkerVillas.h>
#include <memory>
#include <spdlog/spdlog.h>
#include <typeinfo>
#include <villas/node.h>
#include <villas/node/memory.hpp>
#include <villas/node/memory_type.hpp>
#include <villas/path.hpp>
#include <villas/signal_type.hpp>

using namespace CPS;
using namespace villas;

namespace DPsim {

InterfaceVillasQueueless::InterfaceVillasQueueless(
    const String &nodeConfig, const String &name,
    spdlog::level::level_enum logLevel)
    : Interface(name, logLevel), mNodeConfig(nodeConfig), mNode(nullptr),
      mSamplePool(), mSequenceToDpsim(0), mSequenceFromDpsim(0) {}

void InterfaceVillasQueueless::createNode() {
  if (villas::node::memory::init(100) != 0) {
    SPDLOG_LOGGER_ERROR(mLog, "Error: Failed to initialize memory subsystem!");
    std::exit(1);
  }
  json_error_t error;
  json_t *config = json_loads(mNodeConfig.c_str(), 0, &error);
  if (config == nullptr) {
    SPDLOG_LOGGER_ERROR(mLog, "Error: Failed to parse node config! Error: {}",
                        error.text);
    throw JsonError(config, error);
  }

  const json_t *nodeType = json_object_get(config, "type");
  if (nodeType == nullptr) {
    SPDLOG_LOGGER_ERROR(mLog, "Error: Node config does not contain type-key!");
    std::exit(1);
  }
  String nodeTypeString = json_string_value(nodeType);

  mNode = node::NodeFactory::make(nodeTypeString);

  int ret = 0;
  ret = mNode->parse(config);
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(mLog,
                        "Error: Node in InterfaceVillas failed to parse "
                        "config. Parse returned code {}",
                        ret);
    std::exit(1);
  }
  ret = mNode->check();
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(
        mLog,
        "Error: Node in InterfaceVillas failed check. Check returned code {}",
        ret);
    std::exit(1);
  }
  struct villas::node::memory::Type *pool_mt = &villas::node::memory::heap;
  ret = node::pool_init(&mSamplePool, 16,
                        sizeof(node::Sample) + SAMPLE_DATA_LENGTH(64), pool_mt);
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(mLog,
                        "Error: InterfaceVillas failed to init sample pool. "
                        "pool_init returned code {}",
                        ret);
    std::exit(1);
  }

  ret = mNode->prepare();
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(mLog,
                        "Error: Node in InterfaceVillas failed to prepare. "
                        "Prepare returned code {}",
                        ret);
    std::exit(1);
  }
  SPDLOG_LOGGER_INFO(mLog, "Node: {}", mNode->getNameFull());
}

static node::SignalType stdTypeToNodeType(const std::type_info &type) {
  if (type == typeid(Real)) {
    return node::SignalType::FLOAT;
  } else if (type == typeid(Int)) {
    return node::SignalType::INTEGER;
  } else if (type == typeid(Bool)) {
    return node::SignalType::BOOLEAN;
  } else if (type == typeid(Complex)) {
    return node::SignalType::COMPLEX;
  } else {
    return node::SignalType::INVALID;
  }
}

void InterfaceVillasQueueless::createSignals() {
  mNode->out.path = new node::Path();
  mNode->out.path->signals = std::make_shared<node::SignalList>();
  node::SignalList::Ptr nodeOutputSignals =
      mNode->out.path->getOutputSignals(false);
  nodeOutputSignals->clear();
  unsigned int idx = 0;
  for (const auto &[attr, id] : mExportAttrsDpsim) {
    while (id > idx) {
      nodeOutputSignals->push_back(
          std::make_shared<node::Signal>("", "", node::SignalType::INVALID));
      idx++;
    }
    nodeOutputSignals->push_back(std::make_shared<node::Signal>(
        "", "", stdTypeToNodeType(attr->getType())));
  }

  node::SignalList::Ptr nodeInputSignals = mNode->getInputSignals(true);
  if (nodeInputSignals == nullptr) {
    nodeInputSignals = std::make_shared<node::SignalList>();
  } else {
    nodeInputSignals->clear();
  }
  idx = 0;
  for (const auto &[attr, id, blockOnRead, syncOnSimulationStart] :
       mImportAttrsDpsim) {
    while (id > idx) {
      nodeInputSignals->push_back(
          std::make_shared<node::Signal>("", "", node::SignalType::INVALID));
      idx++;
    }
    nodeInputSignals->push_back(std::make_shared<node::Signal>(
        "", "", stdTypeToNodeType(attr->getType())));
    idx++;
  }
}

void InterfaceVillasQueueless::open() {
  createNode();
  createSignals();

  // We have no SuperNode, so just hope type_start doesn't use it...
  mNode->getFactory()->start(nullptr);

  auto ret = mNode->start();
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(mLog,
                        "Fatal error: failed to start node in InterfaceVillas. "
                        "Start returned code {}",
                        ret);
    close();
    std::exit(1);
  }
  mOpened = true;
  mSequenceFromDpsim = 0;
  mSequenceToDpsim = 0;
}

void InterfaceVillasQueueless::close() {
  SPDLOG_LOGGER_INFO(mLog, "Closing InterfaceVillas...");
  int ret = mNode->stop();
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(
        mLog,
        "Error: failed to stop node in InterfaceVillas. Stop returned code {}",
        ret);
    std::exit(1);
  }
  mOpened = false;
  ret = node::pool_destroy(&mSamplePool);
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(mLog,
                        "Error: failed to destroy SamplePool in "
                        "InterfaceVillas. pool_destroy returned code {}",
                        ret);
    std::exit(1);
  }

  mNode->getFactory()->stop();

  delete mNode;
  mOpened = false;
}

CPS::Task::List InterfaceVillasQueueless::getTasks() {
  auto tasks = CPS::Task::List();
  if (!mImportAttrsDpsim.empty()) {
    tasks.push_back(std::make_shared<InterfaceVillasQueueless::PreStep>(*this));
  }
  if (!mExportAttrsDpsim.empty()) {
    tasks.push_back(
        std::make_shared<InterfaceVillasQueueless::PostStep>(*this));
  }
  return tasks;
}

void InterfaceVillasQueueless::PreStep::execute(Real time, Int timeStepCount) {
  auto seqnum = mIntf.readFromVillas();
  if (seqnum != mIntf.mSequenceToDpsim + 1) {
    SPDLOG_LOGGER_WARN(mIntf.mLog, "{} Overrun(s) detected!",
                       seqnum - mIntf.mSequenceToDpsim - 1);
  }
  mIntf.mSequenceToDpsim = seqnum;
}

Int InterfaceVillasQueueless::readFromVillas() {
  node::Sample *sample = nullptr;
  Int seqnum = 0;
  int ret = 0;
  if (mImportAttrsDpsim.size() == 0) {
    return 0;
  }
  try {
    sample = node::sample_alloc(&mSamplePool);
    ret = 0;
    while (ret == 0) {
      ret = mNode->read(&sample, 1);
      if (ret < 0) {
        SPDLOG_LOGGER_ERROR(mLog,
                            "Fatal error: failed to read sample from "
                            "InterfaceVillas. Read returned code {}",
                            ret);
        close();
        std::exit(1);
      } else if (ret == 0) {
        SPDLOG_LOGGER_WARN(mLog,
                           "InterfaceVillas read returned 0. Retrying...");
      }
    }

    if (sample->length != mImportAttrsDpsim.size()) {
      SPDLOG_LOGGER_ERROR(mLog,
                          "Error: Received Sample length ({}) does not match "
                          "configured attributes length ({})",
                          sample->length, mImportAttrsDpsim.size());
      throw RuntimeError(
          "Received Sample length does not match configured attributes length");
    }

    for (size_t i = 0; i < mImportAttrsDpsim.size(); i++) {
      auto attr = std::get<0>(mImportAttrsDpsim[i]);
      if (attr->getType() == typeid(Real)) {
        auto attrReal =
            std::dynamic_pointer_cast<Attribute<Real>>(attr.getPtr());
        attrReal->set(sample->data[i].f);
      } else if (attr->getType() == typeid(Int)) {
        auto attrInt = std::dynamic_pointer_cast<Attribute<Int>>(attr.getPtr());
        attrInt->set(sample->data[i].i);
        if (i == 0) {
          seqnum = sample->data[i].i;
        }
      } else if (attr->getType() == typeid(Bool)) {
        auto attrBool =
            std::dynamic_pointer_cast<Attribute<Bool>>(attr.getPtr());
        attrBool->set(sample->data[i].b);
      } else if (attr->getType() == typeid(Complex)) {
        auto attrComplex =
            std::dynamic_pointer_cast<Attribute<Complex>>(attr.getPtr());
        attrComplex->set(
            Complex(sample->data[i].z.real(), sample->data[i].z.imag()));
      } else {
        SPDLOG_LOGGER_ERROR(mLog, "Error: Unsupported attribute type!");
        throw RuntimeError("Unsupported attribute type!");
      }
    }

    sample_decref(sample);
  } catch (const std::exception &) {
    if (sample)
      sample_decref(sample);

    throw;
  }
  return seqnum;
}

void InterfaceVillasQueueless::PostStep::execute(Real time, Int timeStepCount) {
  mIntf.writeToVillas();
}

void InterfaceVillasQueueless::writeToVillas() {
  if (mExportAttrsDpsim.size() == 0) {
    return;
  }
  node::Sample *sample = nullptr;
  Int ret = 0;
  try {
    sample = node::sample_alloc(&mSamplePool);
    if (sample == nullptr) {
      SPDLOG_LOGGER_ERROR(mLog, "InterfaceVillas could not allocate a new "
                                "sample! Not sending any data!");
      return;
    }

    sample->signals = mNode->getOutputSignals(false);

    for (size_t i = 0; i < mExportAttrsDpsim.size(); i++) {
      auto attr = std::get<0>(mExportAttrsDpsim[i]);
      if (attr->getType() == typeid(Real)) {
        auto attrReal =
            std::dynamic_pointer_cast<Attribute<Real>>(attr.getPtr());
        sample->data[i].f = attrReal->get();
      } else if (attr->getType() == typeid(Int)) {
        auto attrInt = std::dynamic_pointer_cast<Attribute<Int>>(attr.getPtr());
        sample->data[i].i = attrInt->get();
      } else if (attr->getType() == typeid(Bool)) {
        auto attrBool =
            std::dynamic_pointer_cast<Attribute<Bool>>(attr.getPtr());
        sample->data[i].b = attrBool->get();
      } else if (attr->getType() == typeid(Complex)) {
        auto attrComplex =
            std::dynamic_pointer_cast<Attribute<Complex>>(attr.getPtr());
        sample->data[i].z = std::complex<float>(attrComplex->get().real(),
                                                attrComplex->get().imag());
      } else {
        SPDLOG_LOGGER_ERROR(mLog, "Error: Unsupported attribute type!");
        throw RuntimeError("Unsupported attribute type!");
      }
    }

    sample->length = mExportAttrsDpsim.size();
    sample->sequence = mSequenceFromDpsim++;
    sample->flags |= (int)villas::node::SampleFlags::HAS_SEQUENCE;
    sample->flags |= (int)villas::node::SampleFlags::HAS_DATA;
    clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
    sample->flags |= (int)villas::node::SampleFlags::HAS_TS_ORIGIN;

    do {
      ret = mNode->write(&sample, 1);
    } while (ret == 0);
    if (ret < 0)
      SPDLOG_LOGGER_ERROR(mLog,
                          "Failed to write samples to InterfaceVillas. Write "
                          "returned code {}",
                          ret);

    sample_decref(sample);
  } catch (const std::exception &) {
    sample_decref(sample);

    if (ret < 0)
      SPDLOG_LOGGER_ERROR(
          mLog,
          "Failed to write samples to InterfaceVillas. Write returned code {}",
          ret);

    // Don't throw here, because we managed to send something
  }
}

void InterfaceVillasQueueless::syncImports() {
  // Block on read until all attributes with syncOnSimulationStart are read
  mSequenceToDpsim = this->readFromVillas();
}

void InterfaceVillasQueueless::syncExports() {
  // Just push all the attributes
  this->writeToVillas();
}

void InterfaceVillasQueueless::printVillasSignals() const {
  SPDLOG_LOGGER_INFO(mLog, "Export signals:");
  for (const auto &signal : *mNode->getOutputSignals(true)) {
    SPDLOG_LOGGER_INFO(mLog, "Name: {}, Unit: {}, Type: {}", signal->name,
                       signal->unit, node::signalTypeToString(signal->type));
  }

  SPDLOG_LOGGER_INFO(mLog, "Import signals:");
  for (const auto &signal : *mNode->getInputSignals(true)) {
    SPDLOG_LOGGER_INFO(mLog, "Name: {}, Unit: {}, Type: {}", signal->name,
                       signal->unit, node::signalTypeToString(signal->type));
  }
}

} // namespace DPsim
