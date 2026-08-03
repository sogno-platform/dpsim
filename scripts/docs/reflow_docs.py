#!/usr/bin/env python3
# Unwraps documentation prose so each paragraph is one long line, and the sources are uniform.
#
#   python3 scripts/docs/reflow_docs.py --check   # list files that would change
#   python3 scripts/docs/reflow_docs.py --write   # unwrap them
#
# The convention is deliberately NOT to hard-wrap: editors soft-wrap, and a fixed column makes every
# later edit reflow a whole paragraph, which buries a one-word change in a large diff.
#
# Markdown treats a single newline inside a paragraph as a space, so this changes the source and not
# the rendered page. Everything where a line break is significant is left alone: front matter, code
# and math fences, tables, headings, HTML blocks, list items and any line holding a shortcode.

import argparse
import re
import sys
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent.parent / "docs/hugo/content/en/docs"

FENCE = re.compile(r"^\s*(```|~~~)")
HEADING = re.compile(r"^\s{0,3}#{1,6}\s")
TABLE = re.compile(r"^\s*\|")
HTML = re.compile(r"^\s*<")
SHORTCODE = re.compile(r"^\s*\{\{[<%]")
# A shortcode must never be split across lines: Hugo parses it per line and a broken
# {{< ref "..." >}} fails the build with an unterminated quoted string.
HAS_SHORTCODE = re.compile(r"\{\{[<%]")
LIST = re.compile(r"^(\s*(?:[-*+]|\d+\.)\s+)(.*)$")
QUOTE = re.compile(r"^\s*>")
INDENTED_CODE = re.compile(r"^ {4,}\S")


def reflow(text):
    lines = text.split("\n")
    out, i, in_fence, in_front = [], 0, False, False

    # front matter passes through untouched
    if lines and lines[0].strip() == "---":
        out.append(lines[0])
        i = 1
        while i < len(lines) and lines[i].strip() != "---":
            out.append(lines[i])
            i += 1
        if i < len(lines):
            out.append(lines[i])
            i += 1

    para = []

    def flush():
        if not para:
            return
        joined = " ".join(s.strip() for s in para)
        prefix = ""
        m = LIST.match(para[0])
        if m:
            prefix = m.group(1)
            joined = " ".join([m.group(2).strip()] + [s.strip() for s in para[1:]])
        out.append(prefix + joined if prefix else joined)
        para.clear()

    while i < len(lines):
        line = lines[i]
        if FENCE.match(line):
            flush()
            in_fence = not in_fence
            out.append(line)
            i += 1
            continue
        if in_fence:
            out.append(line)
            i += 1
            continue
        if not line.strip():
            flush()
            out.append(line)
            i += 1
            continue
        if (
            HEADING.match(line)
            or TABLE.match(line)
            or HTML.match(line)
            or SHORTCODE.match(line)
            or QUOTE.match(line)
            or INDENTED_CODE.match(line)
        ):
            flush()
            out.append(line)
            i += 1
            continue
        if HAS_SHORTCODE.search(line):
            flush()
            out.append(line)
            i += 1
            continue
        if LIST.match(line) and para:
            flush()
        para.append(line)
        i += 1
    flush()
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    changed = []
    for p in sorted(DOCS.rglob("*.md")):
        original = p.read_text(encoding="utf-8")
        new = reflow(original)
        if new != original:
            changed.append(p.relative_to(DOCS).as_posix())
            if args.write:
                p.write_text(new, encoding="utf-8")

    verb = "unwrapped" if args.write else "would change"
    print(f"{len(changed)} file(s) {verb}")
    if args.check:
        for c in changed:
            print("   ", c)
    return 0


if __name__ == "__main__":
    sys.exit(main())
