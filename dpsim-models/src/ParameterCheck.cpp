// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <sstream>
#include <stdexcept>

#include <dpsim-models/IdentifiedObject.h>
#include <dpsim-models/ParameterCheck.h>

using namespace CPS;

namespace {

template <typename MatrixType>
CheckResult checkInvertible(const MatrixType &value) {
  if (value.rows() != value.cols() || value.rows() == 0)
    return CheckResult::Violated;
  return value.fullPivLu().rank() == value.rows() ? CheckResult::Satisfied
                                                  : CheckResult::Violated;
}

CheckResult fromBool(bool satisfied) {
  return satisfied ? CheckResult::Satisfied : CheckResult::Violated;
}

} // namespace

String CPS::constraintDescription(Constraint constraint) {
  switch (constraint) {
  case Constraint::Finite:
    return "finite";
  case Constraint::NonNegative:
    return "non-negative";
  case Constraint::Positive:
    return "strictly positive";
  case Constraint::NonZero:
    return "non-zero";
  case Constraint::DiagonalPositive:
    return "square with a strictly positive diagonal";
  case Constraint::Invertible:
    return "square and invertible";
  }
  return "valid";
}

CheckResult CPS::satisfies(Real value, Constraint constraint) {
  if (!Math::isFinite(value))
    return CheckResult::Violated;

  switch (constraint) {
  case Constraint::Finite:
    return CheckResult::Satisfied;
  case Constraint::NonNegative:
    return fromBool(value >= 0.);
  case Constraint::Positive:
  case Constraint::DiagonalPositive:
    return fromBool(value > DOUBLE_EPSILON);
  case Constraint::NonZero:
  case Constraint::Invertible:
    return fromBool(std::abs(value) > DOUBLE_EPSILON);
  }
  return CheckResult::Satisfied;
}

CheckResult CPS::satisfies(Complex value, Constraint constraint) {
  if (!Math::isFinite(value))
    return CheckResult::Violated;

  switch (constraint) {
  case Constraint::Finite:
    return CheckResult::Satisfied;
  case Constraint::NonZero:
  case Constraint::Invertible:
    return fromBool(std::abs(value) > DOUBLE_EPSILON);
  case Constraint::NonNegative:
  case Constraint::Positive:
  case Constraint::DiagonalPositive:
    return CheckResult::NotApplicable;
  }
  return CheckResult::Satisfied;
}

CheckResult CPS::satisfies(const Matrix &value, Constraint constraint) {
  if (!value.allFinite())
    return CheckResult::Violated;

  switch (constraint) {
  case Constraint::Finite:
    return CheckResult::Satisfied;
  case Constraint::NonNegative:
    return fromBool((value.array() >= 0.).all());
  case Constraint::Positive:
    return fromBool((value.array() > DOUBLE_EPSILON).all());
  case Constraint::NonZero:
    return fromBool(!value.isZero(DOUBLE_EPSILON));
  case Constraint::DiagonalPositive:
    if (value.rows() != value.cols() || value.rows() == 0)
      return CheckResult::Violated;
    return fromBool(value.diagonal().minCoeff() > DOUBLE_EPSILON);
  case Constraint::Invertible:
    return checkInvertible(value);
  }
  return CheckResult::Satisfied;
}

CheckResult CPS::satisfies(const MatrixComp &value, Constraint constraint) {
  if (!value.allFinite())
    return CheckResult::Violated;

  switch (constraint) {
  case Constraint::Finite:
    return CheckResult::Satisfied;
  case Constraint::NonZero:
    return fromBool(!value.isZero(DOUBLE_EPSILON));
  case Constraint::Invertible:
    return checkInvertible(value);
  case Constraint::NonNegative:
  case Constraint::Positive:
  case Constraint::DiagonalPositive:
    return CheckResult::NotApplicable;
  }
  return CheckResult::Satisfied;
}

String ParameterCheck::resolveName(const AttributeBase::Ptr &attr) const {
  for (const auto &entry : mOwner.attributes())
    if (entry.second.getPtr() == attr.getPtr())
      return entry.first;
  return "<unnamed>";
}

void ParameterCheck::record(const String &parameter, const String &reason) {
  mViolations.push_back({**mOwner.mName, parameter, reason});
}

void ParameterCheck::require(bool condition, const String &reason) {
  if (!condition)
    mViolations.push_back({**mOwner.mName, "", reason});
}

void ParameterCheck::throwIfAny(const Violations &violations) {
  if (violations.empty())
    return;

  std::stringstream message;
  message << "invalid component parameters (" << violations.size()
          << " violation" << (violations.size() == 1 ? "" : "s") << "):";
  for (const auto &violation : violations) {
    message << "\n  " << violation.component;
    if (!violation.parameter.empty())
      message << "." << violation.parameter;
    message << ": " << violation.reason;
  }
  throw std::invalid_argument(message.str());
}
