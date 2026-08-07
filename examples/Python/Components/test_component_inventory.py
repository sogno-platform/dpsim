# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# Constructs every class bound into dpsimpy, catching C++ that is declared but
# never defined and so only fails at import or first use.

import inspect
import types

import pytest

import dpsimpy

# Bound without any constructor, only so subclasses share a Python type.
ABSTRACT = {
    "Attribute",
    "AttributeComplex",
    "AttributeComplexDyn",
    "AttributeComplexStat",
    "AttributeMatrix",
    "AttributeMatrixComp",
    "AttributeMatrixCompDyn",
    "AttributeMatrixCompStat",
    "AttributeMatrixDyn",
    "AttributeMatrixStat",
    "AttributeReal",
    "AttributeRealDyn",
    "AttributeRealStat",
    "AttributeString",
    "AttributeStringDyn",
    "AttributeStringStat",
    "DataLoggerInterface",
    "IdentifiedObject",
    "Interface",
    "MNAStateSpaceExtractor",
    "Math",
    "Scheduler",
    "SimPowerCompComplex",
    "SimPowerCompReal",
    "SimTerminalComplex",
    "SimTerminalReal",
    "TopologicalNode",
    "TopologicalPowerComp",
    "TopologicalTerminal",
    "base.MNASyncGenInterface",
    "base.ReducedOrderSynchronGeneratorComplex",
    "base.ReducedOrderSynchronGeneratorReal",
    "base.Switch",
    "base.SwitchPh3",
    "dp.ph1.ReducedOrderSynchronGeneratorVBR",
    "emt.ph3.ReducedOrderSynchronGeneratorVBR",
    "sp.ph1.ReducedOrderSynchronGeneratorVBR",
    "event.Event",
    "signal.Exciter",
    "signal.ExciterParameters",
    "signal.Governor",
    "signal.GovernorParameters",
    "signal.PSS",
    "signal.PSSParameters",
    "signal.SimSignalComp",
    "signal.TopologicalSignalComp",
    "signal.Turbine",
    "signal.TurbineParameters",
}

# Fields are set after construction, so the constructor takes nothing.
NO_ARG_CONSTRUCTIBLE = {
    "DirectLinearSolverConfiguration",
    "OpenMPLevelScheduler",
    "ThreadLevelScheduler",
    "ThreadListScheduler",
    "signal.ExciterDC1Parameters",
    "signal.ExciterDC1SimpParameters",
    "signal.ExciterST1Parameters",
    "signal.ExciterStaticParameters",
    "signal.HydroGovernorParameters",
    "signal.HydroTurbineParameters",
    "signal.PSS1AParameters",
    "signal.SteamGovernorParameters",
    "signal.SteamTurbineParameters",
}

# Classes bound behind a build flag, so absent from a build without it.
BUILD_OPTIONAL = {
    "dp.ph1.ProfileVoltageSource": "WITH_VILLAS",
    "OpenMPLevelScheduler": "WITH_OPENMP",
}


def _state_space_extractor():
    # Only DP::Ph1 and EMT::Ph3 components reach the contributor factory.
    gnd = dpsimpy.dp.SimNode.gnd
    n1 = dpsimpy.dp.SimNode("n1")
    vs = dpsimpy.dp.ph1.VoltageSource("vs")
    vs.set_parameters(complex(100, 0), 0.0)
    r = dpsimpy.dp.ph1.Resistor("r")
    r.set_parameters(10.0)
    l = dpsimpy.dp.ph1.Inductor("l")
    l.set_parameters(1e-3)
    vs.connect([gnd, n1])
    r.connect([n1, gnd])
    l.connect([n1, gnd])

    sim = dpsimpy.Simulation("inventory_state_space", dpsimpy.LogLevel.off)
    sim.set_system(dpsimpy.SystemTopology(50, [n1], [vs, r, l]))
    sim.set_domain(dpsimpy.Domain.DP)
    sim.set_time_step(1e-4)
    sim.set_final_time(1e-3)
    sim.do_state_space_extraction(True)
    sim.run()
    return sim.get_state_space_extractor()


# Each factory takes a tmp_path and returns a live instance.
CONSTRUCT_WITH_ARGS = {
    "CSVReader": lambda tmp: dpsimpy.CSVReader(
        "reader", str(tmp), {}, dpsimpy.LogLevel.off
    ),
    "RealTimeDataLogger": lambda tmp: dpsimpy.RealTimeDataLogger(
        str(tmp / "rt.csv"), 0.1, 1e-3
    ),
    "StateSpaceModalAnalysis": lambda tmp: dpsimpy.StateSpaceModalAnalysis(
        _state_space_extractor()
    ),
    "SystemTopology": lambda tmp: dpsimpy.SystemTopology(50.0),
    "emt.ph3.SSN_GFM": lambda tmp: dpsimpy.emt.ph3.SSN_GFM("uid", "gfm"),
    "event.SwitchEvent": lambda tmp: dpsimpy.event.SwitchEvent(
        0.1, dpsimpy.dp.ph1.Switch("sw"), True
    ),
    "event.SwitchEvent3Ph": lambda tmp: dpsimpy.event.SwitchEvent3Ph(
        0.1, dpsimpy.emt.ph3.Switch("sw3"), True
    ),
    "sp.ph1.VoltageSourceInverter": lambda tmp: dpsimpy.sp.ph1.VoltageSourceInverter(
        "uid", "vsi"
    ),
}

# Classes this sweep cannot construct, covered by a dedicated test instead.
TESTED_ELSEWHERE = {
    # Needs a valid VILLASnode protobuf sample dump, which only that test builds.
    "dp.ph1.ProfileVoltageSource": (
        "test_newly_bound_components.py::"
        "test_dp_ph1_profile_voltage_source_rejects_malformed_file"
    ),
}


def _walk():
    # Keyed by dotted path with the leading "dpsimpy." stripped.
    classes, enums, seen = {}, {}, set()

    def visit(mod, path):
        if id(mod) in seen:
            return
        seen.add(id(mod))
        for name in sorted(dir(mod)):
            if name.startswith("_"):
                continue
            obj = getattr(mod, name)
            key = f"{path}.{name}".lstrip(".")
            if isinstance(obj, types.ModuleType):
                visit(obj, key)
            elif inspect.isclass(obj):
                # pybind enums carry __members__; plain classes do not.
                (enums if hasattr(obj, "__members__") else classes)[key] = obj

    visit(dpsimpy, "")
    return classes, enums


CLASSES, ENUMS = _walk()

# Everything not otherwise classified must construct from a bare name.
NAME_CONSTRUCTIBLE = sorted(
    set(CLASSES)
    - ABSTRACT
    - NO_ARG_CONSTRUCTIBLE
    - set(CONSTRUCT_WITH_ARGS)
    - set(TESTED_ELSEWHERE)
)


def test_inventory_is_not_empty():
    assert len(CLASSES) > 200
    assert len(ENUMS) > 10


def test_every_class_is_classified():
    # An unclassified binding falls through to the name-constructible default.
    buckets = [
        ABSTRACT,
        NO_ARG_CONSTRUCTIBLE,
        set(CONSTRUCT_WITH_ARGS),
        set(TESTED_ELSEWHERE),
    ]
    overlapping = sorted(
        path for path in CLASSES if sum(1 for bucket in buckets if path in bucket) > 1
    )
    assert overlapping == [], f"classes listed in more than one bucket: {overlapping}"

    known = set(CLASSES) | set(ENUMS)
    optional = set(BUILD_OPTIONAL)
    stale = sorted((set().union(*buckets) - known) - optional)
    assert stale == [], f"buckets reference classes that no longer exist: {stale}"


@pytest.mark.parametrize("path", NAME_CONSTRUCTIBLE)
def test_class_constructs_from_a_name(path):
    obj = CLASSES[path]("test_instance")
    assert obj is not None


@pytest.mark.parametrize("path", sorted(NO_ARG_CONSTRUCTIBLE))
def test_class_constructs_without_arguments(path):
    if path not in CLASSES:
        pytest.skip(f"requires a build with {BUILD_OPTIONAL[path]}")
    obj = CLASSES[path]()
    assert obj is not None


@pytest.mark.parametrize("path", sorted(CONSTRUCT_WITH_ARGS))
def test_class_constructs_with_arguments(path, tmp_path):
    if path not in CLASSES:
        pytest.skip(f"requires a build with {BUILD_OPTIONAL[path]}")
    obj = CONSTRUCT_WITH_ARGS[path](tmp_path)
    assert obj is not None


@pytest.mark.parametrize("path", sorted(ABSTRACT))
def test_abstract_class_rejects_construction(path):
    with pytest.raises(TypeError):
        CLASSES[path]()


@pytest.mark.parametrize("path", sorted(ENUMS))
def test_enum_members_round_trip(path):
    enum = ENUMS[path]
    assert enum.__members__, f"{path} exposes no members"
    for name, member in enum.__members__.items():
        assert getattr(enum, name) == member
        assert enum(int(member)) == member
