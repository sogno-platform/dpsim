"""Fetch example/reference data used by the notebooks, cached via DPSIM_REFERENCE_DATA_DIR in CI."""

import os
import re
import shutil

import requests

_CACHE_ROOT = os.environ.get("DPSIM_REFERENCE_DATA_DIR")
_DOWNLOAD_TIMEOUT_SECONDS = 30

# URL prefix -> local clone dir name under DPSIM_REFERENCE_DATA_DIR
_MIRRORS = [
    (
        "https://raw.githubusercontent.com/dpsim-simulator/reference-results/",
        "reference-results",
    ),
    (
        "https://raw.githubusercontent.com/dpsim-simulator/cim-grid-data/",
        "cim-grid-data",
    ),
    (
        "https://raw.githubusercontent.com/dpsim-simulator/example-profile-data/",
        "example-profile-data",
    ),
    (
        "https://github.com/martinmoraga/dpsim_data/raw/",
        "dpsim_data",
    ),
    (
        "https://git.rwth-aachen.de/acs/public/simulation/dpsim/dpsim-results/raw/",
        "dpsim-results-rwth-gitlab",
    ),
]


def _local_path(url):
    for prefix, local_dir in _MIRRORS:
        if url.startswith(prefix):
            # strip the leading ref segment (branch/tag/commit)
            ref_and_path = url[len(prefix) :]
            _, _, path = ref_and_path.partition("/")
            if path:
                return os.path.join(_CACHE_ROOT, local_dir, path)
    return None


def get_example_file(url, dest=None):
    """Fetch `url` from the local cache if available, else download it; returns the local path."""
    if dest is None:
        dest = os.path.basename(re.sub(r"[?#].*$", "", url))
    dest_dir = os.path.dirname(dest)
    if dest_dir:
        os.makedirs(dest_dir, exist_ok=True)

    if _CACHE_ROOT:
        cached = _local_path(url)
        if cached and os.path.isfile(cached):
            shutil.copyfile(cached, dest)
            return dest

    response = requests.get(url, stream=True, timeout=_DOWNLOAD_TIMEOUT_SECONDS)
    response.raise_for_status()
    with open(dest, "wb") as out_file:
        for chunk in response.iter_content(chunk_size=1 << 16):
            out_file.write(chunk)
    return dest
