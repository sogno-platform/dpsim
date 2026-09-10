#!/usr/bin/env python3
# Regenerates the model availability reference from the component headers.
#
# Run from the repository root:
#   python3 scripts/docs/generate_model_availability.py --check    # fail if the page is stale
#   python3 scripts/docs/generate_model_availability.py --write    # rewrite the page
#
# Every concrete model class must appear in CATEGORIES or in SKIP. A class in neither
# is an error, so adding a model forces a decision about where it is documented.

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
HEADERS = REPO / "dpsim-models/include/dpsim-models"
PAGE = REPO / "docs/hugo/content/en/docs/Reference/model-availability.md"

BEGIN = "<!-- BEGIN GENERATED -->"
END = "<!-- END GENERATED -->"

# Abstract bases, infrastructure and powerflow bus types carry no model of their own.
SKIP = {
    "SSNComp",
    "VTypeSSNComp",
    "ITypeSSNComp",
    "VTypeVariableSSNComp",
    "TwoTerminalVTypeSSNComp",
    "TwoTerminalITypeSSNComp",
    "TwoTerminalVTypeVariableSSNComp",
    "FourTerminalVTypeSSNComp",
    "MixedVTypeVariableSSNComp",
    "ReducedOrderSynchronGeneratorVBR",
    "SynchronGeneratorVBR_Deprecated",
    "DPDQInterface",
    "PQNode",
    "PVNode",
    "VDNode",
}

# Section title -> ordered list of class names. The order here is the order on the page.
CATEGORIES = [
    (
        "Passive elements and sources",
        [
            "Resistor",
            "Inductor",
            "Capacitor",
            "VoltageSource",
            "CurrentSource",
            "VoltageSourceNorton",
            "VoltageSourceRamp",
            "ProfileVoltageSource",
            "ControlledVoltageSource",
            "ControlledCurrentSource",
            "NetworkInjection",
        ],
    ),
    (
        "Branches",
        [
            "PiLine",
            "RxLine",
            "SeriesResistor",
            "Transformer",
            "SolidStateTransformer",
        ],
    ),
    (
        "Switches and loads",
        [
            "Switch",
            "SeriesSwitch",
            "varResSwitch",
            "RXLoad",
            "RXLoadSwitch",
            "PQLoadCS",
            "Load",
            "Shunt",
            "SVC",
        ],
    ),
    (
        "Synchronous generators",
        [
            "SynchronGenerator",
            "SynchronGeneratorDQ",
            "SynchronGeneratorDQODE",
            "SynchronGeneratorDQTrapez",
            "SynchronGeneratorVBR",
            "SynchronGenerator3OrderVBR",
            "SynchronGenerator4OrderVBR",
            "SynchronGenerator5OrderVBR",
            "SynchronGenerator6aOrderVBR",
            "SynchronGenerator6bOrderVBR",
            "SynchronGenerator4OrderPCM",
            "SynchronGenerator6OrderPCM",
            "SynchronGenerator4OrderTPM",
            "SynchronGeneratorIdeal",
            "SynchronGeneratorIter",
            "SynchronGeneratorTrStab",
        ],
    ),
    (
        "Power electronics",
        [
            "AvVoltageSourceInverterDQ",
            "AvVoltSourceInverterStateSpace",
            "Inverter",
            "VoltageSourceInverter",
            "VSIVoltageControlVCO",
            "SSN_GFM",
        ],
    ),
    (
        "State-space nodal components",
        [
            "SSN_Full_Serial_RLC",
            "SSN_Variable_Serial_RLC",
            "SSN_Capacitor",
            "SSN_Inductor",
            "SSNTypeV2T",
            "SSNTypeI2T",
            "PiecewiseLinearInductor",
            "GenericTwoTerminalVTypeSSN",
            "GenericTwoTerminalITypeSSN",
            "GenericFourTerminalVTypeSSN",
        ],
    ),
    (
        "Excitation and stabilizers",
        ["ExciterDC1", "ExciterDC1Simp", "ExciterST1Simp", "ExciterStatic", "PSS1A"],
    ),
    (
        "Turbines and governors",
        [
            "SteamTurbine",
            "SteamTurbineGovernor",
            "HydroTurbine",
            "HydroTurbineGovernor",
            "TurbineGovernor",
            "TurbineGovernorType1",
        ],
    ),
    (
        "Converter control",
        ["PowerControllerVSI", "VoltageControllerVSI", "PLL", "VCO"],
    ),
    (
        "Signal sources and filters",
        [
            "SignalGenerator",
            "SineWaveGenerator",
            "CosineFMGenerator",
            "DCGenerator",
            "FrequencyRampGenerator",
            "FIRFilter",
            "Integrator",
        ],
    ),
    (
        "Decoupling components",
        [
            "DecouplingLine",
            "DecouplingLineEMT",
            "DecouplingLineEMT_Ph3",
            "DecouplingIdealTransformer_SP_Ph1",
            "DecouplingIdealTransformer_DP_Ph1",
            "DecouplingIdealTransformer_EMT_Ph1",
            "DecouplingIdealTransformer_EMT_Ph3",
        ],
    ),
]

# These live in the Signal namespace but are domain specific: the domain is part of the
# class name, not of the directory, so it cannot be recovered from the path.
DOMAIN_OVERRIDES = {
    "DecouplingLine": "DP::Ph1",
    "DecouplingLineEMT": "EMT::Ph1",
    "DecouplingLineEMT_Ph3": "EMT::Ph3",
    "DecouplingIdealTransformer_SP_Ph1": "SP::Ph1",
    "DecouplingIdealTransformer_DP_Ph1": "DP::Ph1",
    "DecouplingIdealTransformer_EMT_Ph1": "EMT::Ph1",
    "DecouplingIdealTransformer_EMT_Ph3": "EMT::Ph3",
}

# Fixed prose emitted under a section heading, before its table or list.
NOTES = {
    "Decoupling components": (
        "These are network components: they connect to nodes and own their own sources. "
        "They are declared in the `Signal` namespace for historical reasons, which is why "
        "their domain appears in the class name rather than in the namespace."
    ),
}

ELECTRICAL = ["SP::Ph1", "SP::Ph3", "DP::Ph1", "DP::Ph3", "EMT::Ph1", "EMT::Ph3"]
YES = "&check;"
NO = "&ndash;"


def scan():
    """class name -> sorted domain list, from the header filenames."""
    found = {}
    for sub in ("SP", "DP", "EMT", "Signal"):
        for h in sorted((HEADERS / sub).glob("*.h")):
            m = re.match(r"^(SP|DP|EMT)_(Ph[13])_(.+)$", h.stem)
            if m:
                cls, domain = m.group(3), f"{m.group(1)}::{m.group(2)}"
            elif sub == "Signal":
                cls, domain = h.stem, "Signal"
            else:
                continue
            if cls in SKIP or cls.startswith("Base_"):
                continue
            found.setdefault(cls, set()).add(DOMAIN_OVERRIDES.get(cls, domain))
    return found


def render(found):
    known = {c for _, classes in CATEGORIES for c in classes}
    uncategorised = sorted(set(found) - known)
    if uncategorised:
        raise SystemExit(
            "These model classes are in neither CATEGORIES nor SKIP, so the page cannot be\n"
            "generated. Add each to a section or to SKIP with a reason:\n  "
            + "\n  ".join(uncategorised)
        )

    header = "| Model | " + " | ".join(ELECTRICAL) + " |"
    rule = "| --- | " + " | ".join([":---:"] * len(ELECTRICAL)) + " |"

    out = [BEGIN, ""]
    for title, classes in CATEGORIES:
        rows = [(c, found[c]) for c in classes if c in found]
        if not rows:
            continue
        out.append(f"## {title}")
        out.append("")
        if title in NOTES:
            out += [NOTES[title], ""]

        # Signal models are domain independent, so a domain matrix would be all one column.
        if all(domains == {"Signal"} for _, domains in rows):
            out += [f"- `{cls}`" for cls, _ in rows]
            out.append("")
            continue

        out += [header, rule]
        for cls, domains in rows:
            cells = [YES if d in domains else NO for d in ELECTRICAL]
            out.append(f"| {cls} | " + " | ".join(cells) + " |")
        out.append("")
    out.append(END)
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    generated = render(scan())
    text = PAGE.read_text(encoding="utf-8")
    if BEGIN not in text or END not in text:
        raise SystemExit(
            f"{PAGE} has no generated block; add {BEGIN} and {END} markers."
        )

    head, rest = text.split(BEGIN, 1)
    _, tail = rest.split(END, 1)
    updated = head + generated + tail

    if args.check:
        if updated != text:
            print(
                f"{PAGE} is out of date; run scripts/docs/generate_model_availability.py --write"
            )
            return 1
        print("model availability page is up to date")
        return 0

    if args.write:
        PAGE.write_text(updated, encoding="utf-8")
        print(f"wrote {PAGE}")
        return 0

    print(generated)
    return 0


if __name__ == "__main__":
    sys.exit(main())
