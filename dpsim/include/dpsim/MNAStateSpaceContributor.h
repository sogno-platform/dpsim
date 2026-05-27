// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <memory>
#include <vector>

#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim/Definitions.h>

namespace DPsim {

/// Live adapter that contributes one component's local state-space blocks to
/// the MNA-coupled state-space extraction.
///
/// The extracted system is assembled as:
///
///   x[k+1]       = AdLocal x[k] + BdMna xMNA[k+1]
///   Y xMNA[k+1] = CdMna x[k]
///
/// where x is the extraction-state vector and xMNA is the full MNA unknown
/// vector. CdMna maps extraction states to MNA current injections.
class MNAStateSpaceContributor {
public:
  using Ptr = std::shared_ptr<MNAStateSpaceContributor>;
  using List = std::vector<Ptr>;

  virtual ~MNAStateSpaceContributor() = default;

  /// Number of local extraction states contributed by this component.
  virtual UInt getStateCount() const = 0;

  /// Returns true if the local matrices may change during simulation.
  virtual Bool isVariable() const { return false; }

  /// Stamp this component's current local state-space contribution.
  virtual void stamp(Matrix &AdLocal, Matrix &BdMna, Matrix &CdMna,
                     UInt stateOffset, UInt mnaVectorSize) const = 0;
};

/// Factory for supported EMT real-valued MNA state-space contributors.
///
/// Components that contribute extraction states return a contributor.
/// Supported algebraic components without extraction states return nullptr.
/// Unsupported components throw an exception.
class MNAStateSpaceContributorFactory {
public:
  static MNAStateSpaceContributor::List
  createList(const CPS::MNAInterface::List &components);

private:
  static MNAStateSpaceContributor::Ptr
  create(const CPS::MNAInterface::Ptr &component);
};

} // namespace DPsim
