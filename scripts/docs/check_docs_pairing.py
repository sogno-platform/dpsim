#!/usr/bin/env python3
# Checks that every Concepts page has an implementation counterpart and vice versa.
#
#   python3 scripts/docs/check_docs_pairing.py
#
# The rule the documentation follows is that a subject is described twice: the mathematics under
# Concepts, naming no class, and the arrangement in code under the Developer Guide. The two link to
# each other. Nothing enforced that until this script, so pages drifted into having only one half.
#
# A page with no counterpart must either gain one or be listed in EXEMPT with a reason.

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
DOCS = REPO / "docs/hugo/content/en/docs"
CONCEPTS = DOCS / "Concepts"
DEVGUIDE = DOCS / "Developer Guide"

# Concepts pages that legitimately have no single implementation page behind them.
EXEMPT_CONCEPTS = {
    "dyn-phasors.md": "a transform underlying every envelope domain, not one component",
    "nodal-analysis.md": "implemented by the solver as a whole; the seam is mnainterface.md",
    "powerflow.md": "paired with powerflow-solvers.md, which links to it by section not page",
    "Models/_index.md": "section landing",
}

# Developer Guide pages that are not the implementation half of a concept page.
EXEMPT_DEVGUIDE = {
    "Architecture/index.md",
    "conventions.md",
    "Attributes/index.md",
    "attribute-usage.md",
    "Scheduling/index.md",
    "adding-tasks.md",
    "initialization.md",
    "mnainterface.md",
    "subcomponents.md",
    "add-model.md",
    "create-simulation.md",
    "debugging.md",
    "Solvers/powerflow-solvers.md",
    "loggers.md",
}


def pages(root):
    return sorted(p for p in root.rglob("*.md") if p.name != "_index.md")


def links_in(path):
    """Every {{< ref "..." >}} target on a page, normalised."""
    text = path.read_text(encoding="utf-8", errors="ignore")
    return {
        m.group(1).strip().lstrip("/")
        for m in re.finditer(r'\{\{<\s*ref\s+"([^"]+)"', text)
    }


def main():
    dev_links = {p: links_in(p) for p in pages(DEVGUIDE)}
    all_dev_targets = set().union(*dev_links.values()) if dev_links else set()

    unpaired_concepts = []
    for p in pages(CONCEPTS):
        rel = p.relative_to(CONCEPTS).as_posix()
        if rel in EXEMPT_CONCEPTS:
            continue
        # A Developer Guide page must reference this page, by full path or by bundle name.
        stem = p.parent.name if p.name == "index.md" else p.stem
        hit = any(
            ("Concepts/" + rel) in t or t.rstrip("/").endswith(stem) or stem in t
            for t in all_dev_targets
        )
        if not hit:
            unpaired_concepts.append(rel)

    unpaired_dev = []
    for p, targets in dev_links.items():
        rel = p.relative_to(DEVGUIDE).as_posix()
        if rel in EXEMPT_DEVGUIDE:
            continue
        if not any("Concepts" in t for t in targets):
            unpaired_dev.append(rel)

    print(
        f"Concepts pages without an implementation counterpart : {len(unpaired_concepts)}"
    )
    for r in unpaired_concepts:
        print(f"    {r}")
    print(f"Developer Guide pages not linked to a concept page   : {len(unpaired_dev)}")
    for r in unpaired_dev:
        print(f"    {r}")

    if unpaired_concepts or unpaired_dev:
        print(
            "\nAdd the missing page and cross-link it, or add an entry to the exemption list "
            "in this script with a reason."
        )
        return 1
    print("\nevery page is paired")
    return 0


if __name__ == "__main__":
    sys.exit(main())
