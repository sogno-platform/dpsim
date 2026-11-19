# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

"""Small checks for the Binder image, separate from the full notebook CI."""

import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"
EXAMPLES = Path(os.environ["DPSIM_CXX_EXAMPLES_DIR"])

assert EXAMPLES == BUILD / "dpsim" / "examples" / "cxx"
assert EXAMPLES.is_dir(), f"C++ examples directory is missing: {EXAMPLES}"
assert (EXAMPLES / "DP_VS_RL1").is_file(), "DP_VS_RL1 was not built"
assert os.access(EXAMPLES / "DP_VS_RL1", os.X_OK), "DP_VS_RL1 is not executable"

# These imports confirm both the compiled extension and the Python package are
# available in the environment used by the notebooks.
import dpsim  # noqa: E402
import dpsimpy  # noqa: E402

assert Path(dpsim.__file__).resolve() == BUILD / "dpsim" / "__init__.py"

# Run a short compiled example from the same repository root notebooks use.
subprocess.run([str(EXAMPLES / "DP_VS_RL1")], cwd=ROOT, check=True)
