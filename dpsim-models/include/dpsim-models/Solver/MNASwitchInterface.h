/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
/// \brief MNA interface to be used by switching devices.
class MNASwitchInterface {
public:
  typedef std::shared_ptr<MNASwitchInterface> Ptr;
  typedef std::vector<Ptr> List;

  virtual ~MNASwitchInterface() = default;

  /// Whether all system-matrix states of the switch can be represented by
  /// precomputed switch matrices.
  /// \note This is expected to be true for implementations of
  /// MNASwitchInterface. The capability is required because some existing
  /// switch components also vary matrix parameters continuously between their
  /// discrete switch states.
  virtual Bool supportsPrecomputedSystemMatrices() const { return true; }

  // #### MNA section ####
  /// Check if switch is closed
  virtual Bool mnaIsClosed() = 0;
  /// Stamps system matrix considering the defined switch position
  virtual void mnaApplySwitchSystemMatrixStamp(Bool closed,
                                               SparseMatrixRow &systemMatrix,
                                               Int freqIdx) final {
    this->mnaCompApplySwitchSystemMatrixStamp(closed, systemMatrix, freqIdx);
    systemMatrix.makeCompressed();
  }
  virtual void mnaCompApplySwitchSystemMatrixStamp(
      Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {}
};
} // namespace CPS
