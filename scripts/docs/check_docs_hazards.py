#!/usr/bin/env python3
# Reports prose that reads like a hazard or a hard constraint but is not inside a callout.
#
#   python3 scripts/docs/check_docs_hazards.py            # list candidates
#   python3 scripts/docs/check_docs_hazards.py --summary  # counts per file only
#
# Anything that will silently cost a reader an afternoon has to break out of the prose, using the
# alert vocabulary documented under Contributing. This script finds sentences that sound like one
# and are not marked. It is a prompt for judgement, not a rule: plenty of matches are ordinary
# descriptive prose and should be left alone.

import argparse
import re
import sys
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent.parent / "docs/hugo/content/en/docs"

# Phrases that tend to mark a hazard, a hard requirement, or a silent failure.
PATTERNS = [
    r"\bsilently\b",
    r"\bwithout warning\b",
    r"\bno warning\b",
    r"\bnothing warns\b",
    r"\bdoes not error\b",
    r"\bnot reported as an error\b",
    r"\balways wrong\b",
    r"\bwrong results\b",
    r"\bincorrect results\b",
    r"\bharder to debug\b",
    r"\bmust be called\b",
    r"\bmust be declared\b",
    r"\bmust be stored\b",
    r"\bis mandatory\b",
    r"\beasy to forget\b",
    r"\beasy to miss\b",
    r"\btrips up\b",
    r"\bcatches people\b",
    r"\bonly .{0,30}\bcan (be logged|participate|cross)\b",
    r"\bcannot be (recorded|logged|exchanged|reached)\b",
    r"\bthe failure to watch for\b",
    r"\bfails at setup\b",
    r"\bruns and is wrong\b",
    r"\bruns but\b.{0,40}\bwrong\b",
    r"\bwill crash\b",
    r"\bsingular\b.{0,40}\bmatrix\b",
    r"\bnot obvious from the output\b",
    r"\blooks like a plausible\b",
    r"\bworth knowing before\b",
    r"\bbefore you start\b",
]
RX = re.compile("|".join(PATTERNS), re.I)

ALERT_OPEN = re.compile(r"\{\{%\s*alert")
ALERT_CLOSE = re.compile(r"\{\{%\s*/\s*alert")
FENCE = re.compile(r"^\s*```")


def scan(path):
    """Yield (line number, text) for hazard-ish prose outside alerts and code fences."""
    inside_alert = inside_fence = False
    for n, line in enumerate(
        path.read_text(encoding="utf-8", errors="ignore").splitlines(), 1
    ):
        if FENCE.match(line):
            inside_fence = not inside_fence
            continue
        if ALERT_OPEN.search(line):
            inside_alert = True
        if ALERT_CLOSE.search(line):
            inside_alert = False
            continue
        if inside_alert or inside_fence:
            continue
        if RX.search(line):
            yield n, line.strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", action="store_true")
    args = ap.parse_args()

    total = 0
    for p in sorted(DOCS.rglob("*.md")):
        hits = list(scan(p))
        if not hits:
            continue
        total += len(hits)
        rel = p.relative_to(DOCS).as_posix()
        if args.summary:
            print(f"{len(hits):3d}  {rel}")
            continue
        print(f"\n{rel}")
        for n, text in hits:
            print(f"  {n:4d}  {text[:150]}")

    print(
        f"\n{total} unmarked candidate(s). Mark the ones that are genuine hazards with the alert "
        f"vocabulary; leave descriptive prose alone."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
