# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# Sweeps every class bound into dpsimpy and asserts it is actually usable.
# Catches components whose C++ is declared but never compiled or defined, which
# links fine into a Python module and only fails at import or first use.

import inspect

import pytest

import dpsimpy

# Classes that intentionally cannot be built from a bare name. Anything not
# listed here must construct as Cls("name"), so a new unusable binding fails.
NOT_NAME_CONSTRUCTIBLE = {
    # Abstract bases, exposed only so concrete subclasses share a Python type.
    "sp.ph1.ReducedOrderSynchronGeneratorVBR": "abstract base",
    "dp.ph1.ReducedOrderSynchronGeneratorVBR": "abstract base",
    "emt.ph3.ReducedOrderSynchronGeneratorVBR": "abstract base",
    "signal.Exciter": "abstract base",
    "signal.Governor": "abstract base",
    "signal.PSS": "abstract base",
    "signal.Turbine": "abstract base",
    "signal.SimSignalComp": "abstract base",
    "signal.TopologicalSignalComp": "abstract base",
    "signal.ExciterParameters": "abstract parameter base",
    "signal.GovernorParameters": "abstract parameter base",
    "signal.PSSParameters": "abstract parameter base",
    "signal.TurbineParameters": "abstract parameter base",
    # Parameter structs: every field is required, there is no name argument.
    "signal.ExciterDC1Parameters": "parameter struct",
    "signal.ExciterDC1SimpParameters": "parameter struct",
    "signal.ExciterST1Parameters": "parameter struct",
    "signal.ExciterStaticParameters": "parameter struct",
    "signal.HydroGovernorParameters": "parameter struct",
    "signal.HydroTurbineParameters": "parameter struct",
    "signal.PSS1AParameters": "parameter struct",
    "signal.SteamGovernorParameters": "parameter struct",
    "signal.SteamTurbineParameters": "parameter struct",
    # Components whose parameters are fixed at construction.
    "sp.ph1.VoltageSourceInverter": "requires uid and name",
    "dp.ph1.ProfileVoltageSource": "requires a source file",
    "emt.ph3.SSN_GFM": "requires model matrices",
}

# Allowlist entries bound behind a build flag, so absent from a build without it.
BUILD_OPTIONAL = {
    "dp.ph1.ProfileVoltageSource",  # WITH_VILLAS
}


def _submodules():
    mods = []
    for domain in ("sp", "dp", "emt"):
        dom = getattr(dpsimpy, domain)
        for phase in ("ph1", "ph3"):
            if hasattr(dom, phase):
                mods.append((f"{domain}.{phase}", getattr(dom, phase)))
    mods.append(("signal", dpsimpy.signal))
    return mods


def _all_classes():
    found = []
    for path, mod in _submodules():
        for name in sorted(dir(mod)):
            if name.startswith("_"):
                continue
            obj = getattr(mod, name)
            if inspect.isclass(obj):
                found.append((f"{path}.{name}", obj))
    return found


ALL_CLASSES = _all_classes()


def test_inventory_is_not_empty():
    # Guards against the sweep silently collecting nothing.
    assert len(ALL_CLASSES) > 100


@pytest.mark.parametrize("path,cls", ALL_CLASSES, ids=[p for p, _ in ALL_CLASSES])
def test_class_is_constructible(path, cls):
    if path in NOT_NAME_CONSTRUCTIBLE:
        pytest.skip(NOT_NAME_CONSTRUCTIBLE[path])
    obj = cls("test_instance")
    assert obj is not None


def test_allowlist_has_no_stale_entries():
    # A component that gains a name constructor must leave the allowlist.
    known = {path for path, _ in ALL_CLASSES}
    stale = sorted(set(NOT_NAME_CONSTRUCTIBLE) - known - BUILD_OPTIONAL)
    assert stale == [], f"allowlist references classes that no longer exist: {stale}"

    wrongly_listed = []
    for path, cls in ALL_CLASSES:
        if path not in NOT_NAME_CONSTRUCTIBLE:
            continue
        try:
            cls("test_instance")
        except TypeError:
            continue
        wrongly_listed.append(path)
    assert (
        wrongly_listed == []
    ), f"these now construct from a name and should leave the allowlist: {wrongly_listed}"
