// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <chrono>
#include <poll.h>
#include <thread>
#include <unistd.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim-models/Logger.h>
#include <dpsim-villas/InterfaceWorkerVillas.h>
#include <villas/path.hpp>
#include <villas/signal_list.hpp>

using namespace CPS;
using namespace DPsim;

Bool InterfaceWorkerVillas::villasInitialized = false;
UInt InterfaceWorkerVillas::villasAffinity = 0;
UInt InterfaceWorkerVillas::villasPriority = 0;
UInt InterfaceWorkerVillas::villasHugePages = 100;

InterfaceWorkerVillas::InterfaceWorkerVillas(const String &nodeConfig,
                                             UInt queueLength,
                                             UInt sampleLength,
                                             spdlog::level::level_enum logLevel)
    : InterfaceWorker(), mNodeConfig(nodeConfig), mQueueLength(queueLength),
      mSampleLength(sampleLength) {
  mLog = CPS::Logger::get("InterfaceWorkerVillas", logLevel, logLevel);
}

void InterfaceWorkerVillas::open() {
  SPDLOG_LOGGER_INFO(mLog, "Opening InterfaceWorkerVillas...");
  Log::getInstance().setLevel(mLog->level());

  if (!InterfaceWorkerVillas::villasInitialized) {
    SPDLOG_LOGGER_INFO(mLog, "Initializing Villas...");
    initVillas();
    InterfaceWorkerVillas::villasInitialized = true;
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

  SPDLOG_LOGGER_INFO(mLog, "Preparing VILLASNode instance...");
  setupNodeSignals();
  prepareNode();
  SPDLOG_LOGGER_INFO(mLog, "Node is ready to send / receive data!");
  mOpened = true;

  mSequence = 0;
  mLastSample = node::sample_alloc(&mSamplePool);
  mLastSample->signals = mNode->getInputSignals(false);
  mLastSample->sequence = 0;
  mLastSample->ts.origin.tv_sec = 0;
  mLastSample->ts.origin.tv_nsec = 0;

  std::memset(&mLastSample->data, 0, mLastSample->capacity * sizeof(float));
}

void InterfaceWorkerVillas::prepareNode() {
  int ret = node::pool_init(&mSamplePool, mQueueLength,
                            sizeof(Sample) + SAMPLE_DATA_LENGTH(mSampleLength));
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

  mNode->getFactory()->start(
      nullptr); //We have no SuperNode, so just hope type_start doesnt use it...

  ret = mNode->start();
  if (ret < 0) {
    SPDLOG_LOGGER_ERROR(mLog,
                        "Fatal error: failed to start node in InterfaceVillas. "
                        "Start returned code {}",
                        ret);
    close();
    std::exit(1);
  }
}

void InterfaceWorkerVillas::setupNodeSignals() {
  mNode->out.path = new node::Path();
  mNode->out.path->signals = std::make_shared<node::SignalList>();
  node::SignalList::Ptr nodeOutputSignals =
      mNode->out.path->getOutputSignals(false);
  nodeOutputSignals->clear();
  int idx = 0;
  for (const auto &[id, signal] : mExportSignals) {
    while (id > idx) {
      nodeOutputSignals->push_back(
          std::make_shared<node::Signal>("", "", node::SignalType::INVALID));
      idx++;
    }
    nodeOutputSignals->push_back(signal);
    idx++;
  }

  node::SignalList::Ptr nodeInputSignals = mNode->getInputSignals(true);
  if (nodeInputSignals == nullptr) {
    nodeInputSignals = std::make_shared<node::SignalList>();
  } else {
    nodeInputSignals->clear();
  }
  idx = 0;
  for (const auto &[id, signal] : mImportSignals) {
    while (id > idx) {
      nodeInputSignals->push_back(
          std::make_shared<node::Signal>("", "", node::SignalType::INVALID));
      idx++;
    }
    nodeInputSignals->push_back(signal);
    idx++;
  }
}

void InterfaceWorkerVillas::close() {
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
}

void InterfaceWorkerVillas::readValuesFromEnv(
    std::vector<InterfaceQueued::AttributePacket> &updatedAttrs) {
  Sample *sample = nullptr;
  int ret = 0;
  bool shouldRead = false;
  try {
    auto pollFds = mNode->getPollFDs();

    if (!pollFds.empty()) {
      auto pfds = std::vector<struct pollfd>();

      for (auto pollFd : pollFds) {
        pfds.push_back(pollfd{.fd = pollFd, .events = POLLIN});
      }

      ret = ::poll(pfds.data(), pfds.size(), 0);

      if (ret < 0) {
        SPDLOG_LOGGER_ERROR(mLog,
                            "Fatal error: failed to read sample from "
                            "InterfaceVillas. Poll returned code {}",
                            ret);
        close();
        std::exit(1);
      }

      if (ret == 0) {
        return;
      }

      for (const auto &pfd : pfds) {
        if (pfd.revents & POLLIN) {
          shouldRead = true;
          break;
        }
      }
    } else {
      //If the node does not support pollFds just do a blocking read
      shouldRead = true;
    }

    if (shouldRead) {
      sample = node::sample_alloc(&mSamplePool);

      ret = mNode->read(&sample, 1);
      if (ret < 0) {
        SPDLOG_LOGGER_ERROR(mLog,
                            "Fatal error: failed to read sample from "
                            "InterfaceVillas. Read returned code {}",
                            ret);
        close();
        std::exit(1);
      }

      if (ret != 0) {
        for (UInt i = 0; i < mImports.size(); i++) {
          auto importedAttr = std::get<0>(mImports[i])(sample);
          if (!importedAttr.isNull()) {
            updatedAttrs.emplace_back(InterfaceQueued::AttributePacket{
                importedAttr, i, mCurrentSequenceInterfaceToDpsim,
                InterfaceQueued::AttributePacketFlags::PACKET_NO_FLAGS});
            mCurrentSequenceInterfaceToDpsim++;
          }
        }

        if (!pollFds.empty()) {
          //Manually clear the event file descriptor since Villas does not do that for some reason
          //See https://github.com/VILLASframework/node/issues/309
          uint64_t result = 0;
          ret = (int)::read(pollFds[0], &result, 8);
          if (ret < 0) {
            SPDLOG_LOGGER_WARN(
                mLog, "Could not reset poll file descriptor! Read returned {}",
                ret);
          }
          if (result > 1) {
            result = result - 1;
            ret = (int)::write(pollFds[0], (void *)&result, 8);
            if (ret < 0) {
              SPDLOG_LOGGER_WARN(
                  mLog,
                  "Could not decrement poll file descriptor! Write returned {}",
                  ret);
            }
          }
        }
      }

      sample_decref(sample);
    }
  } catch (const std::exception &) {
    if (sample)
      sample_decref(sample);

    throw;
  }
}

void InterfaceWorkerVillas::writeValuesToEnv(
    std::vector<InterfaceQueued::AttributePacket> &updatedAttrs) {
  // Update export sequence IDs
  for (const auto &packet : updatedAttrs) {
    if (std::get<1>(mExports[packet.attributeId]) < packet.sequenceId) {
      std::get<1>(mExports[packet.attributeId]) = packet.sequenceId;
    }
  }

  // Remove outdated packets
  auto beginOutdated = std::remove_if(
      updatedAttrs.begin(), updatedAttrs.end(), [this](auto packet) {
        return std::get<1>(mExports[packet.attributeId]) > packet.sequenceId;
      });
  updatedAttrs.erase(beginOutdated, updatedAttrs.end());

  Sample *sample = nullptr;
  Int ret = 0;
  bool done = false;
  bool sampleFilled = false;
  try {
    sample = node::sample_alloc(&mSamplePool);
    if (sample == nullptr) {
      SPDLOG_LOGGER_ERROR(mLog, "InterfaceVillas could not allocate a new "
                                "sample! Not sending any data!");
      return;
    }

    sample->signals = mNode->getOutputSignals(false);
    auto beginExported = std::remove_if(
        updatedAttrs.begin(), updatedAttrs.end(),
        [this, &sampleFilled, &sample](auto packet) {
          if (!std::get<2>(mExports[packet.attributeId])) {
            //Write attribute to sample ASAP
            std::get<0>(mExports[packet.attributeId])(packet.value, sample);
            sampleFilled = true;
            return true;
          }
          return false;
        });
    updatedAttrs.erase(beginExported, updatedAttrs.end());

    //Check if the remaining packets form a complete set
    if (((long)updatedAttrs.size()) ==
        std::count_if(mExports.cbegin(), mExports.cend(),
                      [](auto x) { return std::get<2>(x); })) {
      for (const auto &packet : updatedAttrs) {
        std::get<0>(mExports[packet.attributeId])(packet.value, sample);
      }
      sampleFilled = true;
      updatedAttrs.clear();
    }

    if (sampleFilled) {
      sample->sequence = mSequence++;
      sample->flags |= (int)villas::node::SampleFlags::HAS_SEQUENCE;
      sample->flags |= (int)villas::node::SampleFlags::HAS_DATA;
      clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
      sample->flags |= (int)villas::node::SampleFlags::HAS_TS_ORIGIN;
      done = true;

      do {
        ret = mNode->write(&sample, 1);
      } while (ret == 0);
      if (ret < 0)
        SPDLOG_LOGGER_ERROR(mLog,
                            "Failed to write samples to InterfaceVillas. Write "
                            "returned code {}",
                            ret);

      sample_copy(mLastSample, sample);
    }
    sample_decref(sample);
  } catch (const std::exception &) {
    /* We need to at least send something, so determine where exactly the
		 * timer expired and either resend the last successfully sent sample or
		 * just try to send this one again.
		 * TODO: can this be handled better? */
    if (!done)
      sample = mLastSample;

    while (ret == 0)
      ret = mNode->write(&sample, 1);

    sample_decref(sample);

    if (ret < 0)
      SPDLOG_LOGGER_ERROR(
          mLog,
          "Failed to write samples to InterfaceVillas. Write returned code {}",
          ret);

    /* Don't throw here, because we managed to send something */
  }
}

void InterfaceWorkerVillas::initVillas() const {
  if (int ret = node::memory::init(villasHugePages); ret)
    throw RuntimeError("Error: VillasNode failed to initialize memory system");

  villas::kernel::rt::init(villasPriority, villasAffinity);
}

void InterfaceWorkerVillas::configureExport(UInt attributeId,
                                            const std::type_info &type,
                                            UInt idx, Bool waitForOnWrite,
                                            const String &name,
                                            const String &unit) {
  if (mOpened) {
    if (mLog != nullptr) {
      SPDLOG_LOGGER_WARN(mLog, "InterfaceVillas has already been opened! "
                               "Configuration will remain unchanged.");
    }
    return;
  }
  if (attributeId != mExports.size()) {
    if (mLog != nullptr) {
      SPDLOG_LOGGER_WARN(
          mLog, "The exports already configured do not match with the given "
                "attribute ID! Configuration will remain unchanged.");
    }
    return;
  }

  if (type == typeid(Int)) {
    mExports.emplace_back(
        [idx](AttributeBase::Ptr attr, Sample *smp) {
          if (idx >= smp->capacity)
            throw std::out_of_range("not enough space in allocated sample");
          if (idx >= smp->length)
            smp->length = idx + 1;

          Attribute<Int>::Ptr attrTyped =
              std::dynamic_pointer_cast<Attribute<Int>>(attr.getPtr());

          if (attrTyped.isNull())
            throw InvalidAttributeException();

          smp->data[idx].i = **attrTyped;
        },
        0, waitForOnWrite);
    mExportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::INTEGER);
  } else if (type == typeid(Real)) {
    mExports.emplace_back(
        [idx](AttributeBase::Ptr attr, Sample *smp) {
          if (idx >= smp->capacity)
            throw std::out_of_range("not enough space in allocated sample");
          if (idx >= smp->length)
            smp->length = idx + 1;

          Attribute<Real>::Ptr attrTyped =
              std::dynamic_pointer_cast<Attribute<Real>>(attr.getPtr());

          if (attrTyped.isNull())
            throw InvalidAttributeException();

          smp->data[idx].f = **attrTyped;
        },
        0, waitForOnWrite);
    mExportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::FLOAT);
  } else if (type == typeid(Complex)) {
    mExports.emplace_back(
        [idx](AttributeBase::Ptr attr, Sample *smp) {
          if (idx >= smp->capacity)
            throw std::out_of_range("not enough space in allocated sample");
          if (idx >= smp->length)
            smp->length = idx + 1;

          Attribute<Complex>::Ptr attrTyped =
              std::dynamic_pointer_cast<Attribute<Complex>>(attr.getPtr());

          if (attrTyped.isNull())
            throw InvalidAttributeException();

          smp->data[idx].z = **attrTyped;
        },
        0, waitForOnWrite);
    mExportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::COMPLEX);
  } else if (type == typeid(Bool)) {
    mExports.emplace_back(
        [idx](AttributeBase::Ptr attr, Sample *smp) {
          if (idx >= smp->capacity)
            throw std::out_of_range("not enough space in allocated sample");
          if (idx >= smp->length)
            smp->length = idx + 1;

          Attribute<Bool>::Ptr attrTyped =
              std::dynamic_pointer_cast<Attribute<Bool>>(attr.getPtr());

          if (attrTyped.isNull())
            throw InvalidAttributeException();

          smp->data[idx].b = **attrTyped;
        },
        0, waitForOnWrite);
    mExportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::BOOLEAN);
  } else {
    if (mLog != nullptr) {
      SPDLOG_LOGGER_WARN(mLog, "Unsupported attribute type! Interface "
                               "configuration will remain unchanged!");
    }
  }
}

void InterfaceWorkerVillas::configureImport(UInt attributeId,
                                            const std::type_info &type,
                                            UInt idx, const String &name,
                                            const String &unit) {
  if (mOpened) {
    if (mLog != nullptr) {
      SPDLOG_LOGGER_WARN(mLog, "InterfaceVillas has already been opened! "
                               "Configuration will remain unchanged.");
    }
    return;
  }
  if (attributeId != mImports.size()) {
    if (mLog != nullptr) {
      SPDLOG_LOGGER_WARN(
          mLog, "The imports already configured do not match with the given "
                "attribute ID! Configuration will remain unchanged.");
    }
    return;
  }
  const auto &log = mLog;

  if (type == typeid(Int)) {
    mImports.emplace_back(
        [idx, log](Sample *smp) -> AttributeBase::Ptr {
          if (idx >= smp->length) {
            log->error("incomplete data received from InterfaceVillas");
            return nullptr;
          }
          return AttributePointer<AttributeBase>(
              AttributeStatic<Int>::make(smp->data[idx].i));
        },
        0);
    mImportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::INTEGER);
  } else if (type == typeid(Real)) {
    mImports.emplace_back(
        [idx, log](Sample *smp) -> AttributeBase::Ptr {
          if (idx >= smp->length) {
            log->error("incomplete data received from InterfaceVillas");
            return nullptr;
          }
          return AttributePointer<AttributeBase>(
              AttributeStatic<Real>::make(smp->data[idx].f));
        },
        0);
    mImportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::FLOAT);
  } else if (type == typeid(Complex)) {
    mImports.emplace_back(
        [idx, log](Sample *smp) -> AttributeBase::Ptr {
          if (idx >= smp->length) {
            log->error("incomplete data received from InterfaceVillas");
            return nullptr;
          }
          return AttributePointer<AttributeBase>(
              AttributeStatic<Complex>::make(smp->data[idx].z));
        },
        0);
    mImportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::COMPLEX);
  } else if (type == typeid(Bool)) {
    mImports.emplace_back(
        [idx, log](Sample *smp) -> AttributeBase::Ptr {
          if (idx >= smp->length) {
            log->error("incomplete data received from InterfaceVillas");
            return nullptr;
          }
          return AttributePointer<AttributeBase>(
              AttributeStatic<Bool>::make(smp->data[idx].b));
        },
        0);
    mImportSignals[idx] =
        std::make_shared<node::Signal>(name, unit, node::SignalType::BOOLEAN);
  } else {
    if (mLog != nullptr) {
      SPDLOG_LOGGER_WARN(mLog, "Unsupported attribute type! Interface "
                               "configuration will remain unchanged!");
    }
  }
}

void InterfaceWorkerVillas::printSignals() const {
  SPDLOG_LOGGER_INFO(
      mLog,
      "InterfaceWorkerVillas Settings: Queue Length: {}, Sample Length: {}",
      mQueueLength, mSampleLength);
  SPDLOG_LOGGER_INFO(mLog, "Export signals:");
  for (const auto &[id, signal] : mExportSignals) {
    SPDLOG_LOGGER_INFO(mLog, "ID: {}, Name: {}, Unit: {}, Type: {}", id,
                       signal->name, signal->unit,
                       node::signalTypeToString(signal->type));
  }

  SPDLOG_LOGGER_INFO(mLog, "Import signals:");
  for (const auto &[id, signal] : mImportSignals) {
    SPDLOG_LOGGER_INFO(mLog, "ID: {}, Name: {}, Unit: {}, Type: {}", id,
                       signal->name, signal->unit,
                       node::signalTypeToString(signal->type));
  }
}
