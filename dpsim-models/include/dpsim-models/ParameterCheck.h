// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <initializer_list>
#include <vector>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/MathUtils.h>

namespace CPS {

class IdentifiedObject;

enum class Constraint {
  Finite,
  NonNegative,
  Positive,
  NonZero,
  DiagonalPositive,
  Invertible,
};

enum class CheckResult {
  Satisfied,
  Violated,
  NotApplicable,
};

String constraintDescription(Constraint constraint);

CheckResult satisfies(Real value, Constraint constraint);
CheckResult satisfies(Complex value, Constraint constraint);
CheckResult satisfies(const Matrix &value, Constraint constraint);
CheckResult satisfies(const MatrixComp &value, Constraint constraint);

class ParameterCheck {
public:
  using ConstraintList = std::initializer_list<Constraint>;

  struct Violation {
    String component;
    String parameter;
    String reason;
  };

  using Violations = std::vector<Violation>;

  ParameterCheck(const IdentifiedObject &owner, Violations &violations)
      : mOwner(owner), mViolations(violations) {}

  template <typename T>
  void operator()(const AttributePointer<Attribute<T>> &attr,
                  ConstraintList constraints) {
    if (attr.isNull())
      return;

    const T &value = **attr;
    for (auto constraint : constraints) {
      switch (satisfies(value, constraint)) {
      case CheckResult::Satisfied:
        break;
      case CheckResult::Violated:
        record(resolveName(attr),
               "must be " + constraintDescription(constraint));
        break;
      case CheckResult::NotApplicable:
        record(resolveName(attr),
               "constraint '" + constraintDescription(constraint) +
                   "' does not apply to this parameter type");
        break;
      }
    }
  }

  void require(bool condition, const String &reason);

  static void throwIfAny(const Violations &violations);

private:
  String resolveName(const AttributeBase::Ptr &attr) const;
  void record(const String &parameter, const String &reason);

  const IdentifiedObject &mOwner;
  Violations &mViolations;
};
} // namespace CPS
