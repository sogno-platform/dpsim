/* An interface for data loggers for simulation data
 * logging.
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Filesystem.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/Task.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <iomanip>

namespace DPsim {

class DataLoggerInterface {
protected:
  std::map<String, CPS::AttributeBase::Ptr> mAttributes;

public:
  typedef std::shared_ptr<DataLoggerInterface> Ptr;
  typedef std::vector<DataLoggerInterface::Ptr> List;

  DataLoggerInterface() : mAttributes(){};

  // Start the logger. After starting the number of columns should not be changed.
  virtual void start() = 0;
  // Stops the logger. Afterwards it should not be used anymore.
  virtual void stop() = 0;

  virtual void logAttribute(const String &name, CPS::AttributeBase::Ptr attr, UInt rowsMax = 0, UInt colsMax = 0) {
    if (auto attrReal = std::dynamic_pointer_cast<CPS::Attribute<Real>>(attr.getPtr())) {
      mAttributes[name] = attrReal;
    } else if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<Complex>>(attr.getPtr())) {
      mAttributes[name + ".re"] = attrComp->deriveReal();
      mAttributes[name + ".im"] = attrComp->deriveImag();
    } else if (auto attrInt = std::dynamic_pointer_cast<CPS::Attribute<Int>>(attr.getPtr())) {
      mAttributes[name] = attrInt;
    } else if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<Matrix>>(attr.getPtr())) {
      UInt rows = static_cast<UInt>((**attrMatrix).rows());
      UInt cols = static_cast<UInt>((**attrMatrix).cols());
      if (rowsMax == 0 || rowsMax > rows)
        rowsMax = rows;
      if (colsMax == 0 || colsMax > cols)
        colsMax = cols;
      if (rows == 1 && cols == 1) {
        mAttributes[name] = attrMatrix->deriveCoeff<Real>(0, 0);
      } else if (cols == 1) {
        for (UInt k = 0; k < rowsMax; ++k) {
          mAttributes[name + "_" + std::to_string(k)] = attrMatrix->deriveCoeff<Real>(k, 0);
        }
      } else {
        for (UInt k = 0; k < rowsMax; ++k) {
          for (UInt l = 0; l < colsMax; ++l) {
            mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l)] = attrMatrix->deriveCoeff<Real>(k, l);
          }
        }
      }
    } else if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<MatrixComp>>(attr.getPtr())) {
      UInt rows = static_cast<UInt>((**attrMatrix).rows());
      UInt cols = static_cast<UInt>((**attrMatrix).cols());
      if (rowsMax == 0 || rowsMax > rows)
        rowsMax = rows;
      if (colsMax == 0 || colsMax > cols)
        colsMax = cols;
      if (rows == 1 && cols == 1) {
        mAttributes[name + ".re"] = attrMatrix->deriveCoeff<Complex>(0, 0)->deriveReal();
        mAttributes[name + ".im"] = attrMatrix->deriveCoeff<Complex>(0, 0)->deriveImag();
      } else if (cols == 1) {
        for (UInt k = 0; k < rowsMax; ++k) {
          mAttributes[name + "_" + std::to_string(k) + ".re"] = attrMatrix->deriveCoeff<Complex>(k, 0)->deriveReal();
          mAttributes[name + "_" + std::to_string(k) + ".im"] = attrMatrix->deriveCoeff<Complex>(k, 0)->deriveImag();
        }
      } else {
        for (UInt k = 0; k < rowsMax; ++k) {
          for (UInt l = 0; l < colsMax; ++l) {
            mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l) + ".re"] = attrMatrix->deriveCoeff<Complex>(k, l)->deriveReal();
            mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l) + ".im"] = attrMatrix->deriveCoeff<Complex>(k, l)->deriveImag();
          }
        }
      }
    } else {
      throw std::runtime_error("DataLoggerInterface: Unknown attribute type for attribute " + name);
    }
  }

  /// DEPRECATED: Only use for compatiblity, otherwise this just adds extra overhead to the logger. Instead just call logAttribute multiple times for every coefficient using
  /// `attr->deriveCoeff<>(a,b)`.
  void logAttribute(const std::vector<String> &name, CPS::AttributeBase::Ptr attr) {
    if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<Matrix>>(attr.getPtr())) {
      if ((**attrMatrix).rows() == 1 && (**attrMatrix).cols() == 1) {
        logAttribute(name[0], attrMatrix->deriveCoeff<CPS::Real>(0, 0));
      } else if ((**attrMatrix).cols() == 1) {
        for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
          logAttribute(name[k], attrMatrix->deriveCoeff<CPS::Real>(k, 0));
        }
      } else {
        for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
          for (UInt l = 0; l < (**attrMatrix).cols(); ++l) {
            logAttribute(name[k * (**attrMatrix).cols() + l], attrMatrix->deriveCoeff<CPS::Real>(k, l));
          }
        }
      }
    } else if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<MatrixComp>>(attr.getPtr())) {
      if ((**attrMatrix).rows() == 1 && (**attrMatrix).cols() == 1) {
        logAttribute(name[0], attrMatrix->deriveCoeff<CPS::Complex>(0, 0));
      } else if ((**attrMatrix).cols() == 1) {
        for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
          logAttribute(name[k], attrMatrix->deriveCoeff<CPS::Complex>(k, 0));
        }
      } else {
        for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
          for (UInt l = 0; l < (**attrMatrix).cols(); ++l) {
            logAttribute(name[k * (**attrMatrix).cols() + l], attrMatrix->deriveCoeff<CPS::Complex>(k, l));
          }
        }
      }
    }
  }

  virtual void log(Real time, Int timeStepCount) = 0;

  virtual CPS::Task::Ptr getTask() = 0;
};
} // namespace DPsim
