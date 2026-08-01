# Generates the tutorial circuit diagrams. Colours are chosen to read on light and dark grounds,
# since an <img>-embedded SVG cannot inherit the page's currentColor.
import pathlib, sys

INK = "#5b6a72"
ACC = "#0c6b75"
HEAD = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" '
    'font-family="system-ui,sans-serif" font-size="13">'
)
STYLE = f'<g fill="none" stroke="{INK}" stroke-width="2" stroke-linecap="round">'


def wire(*pts):
    return f'<polyline points="{" ".join(f"{x},{y}" for x, y in pts)}"/>'


def source(x, y, label):
    return (
        f'<circle cx="{x}" cy="{y}" r="18"/>'
        f'<path d="M {x-9} {y} q 4.5 -7 9 0 t 9 0"/>'
        f'<text x="{x-30}" y="{y+4}" text-anchor="end" stroke="none" fill="{INK}">{label}</text>'
    )


def resistor(x, y, label, vertical=False):
    if vertical:
        body = f'<rect x="{x-11}" y="{y-22}" width="22" height="44" rx="2"/>'
        lab = f'<text x="{x+20}" y="{y+4}" stroke="none" fill="{INK}">{label}</text>'
    else:
        body = f'<rect x="{x-22}" y="{y-11}" width="44" height="22" rx="2"/>'
        lab = f'<text x="{x}" y="{y-20}" text-anchor="middle" stroke="none" fill="{INK}">{label}</text>'
    return body + lab


def inductor(x, y, label, vertical=True):
    if vertical:
        d = f"M {x} {y-24} " + " ".join(f"a 8 8 0 0 1 0 12" for _ in range(4))
        lab = f'<text x="{x+18}" y="{y+4}" stroke="none" fill="{INK}">{label}</text>'
    else:
        d = f"M {x-24} {y} " + " ".join(f"a 8 8 0 0 1 12 0" for _ in range(4))
        lab = f'<text x="{x}" y="{y-18}" text-anchor="middle" stroke="none" fill="{INK}">{label}</text>'
    return f'<path d="{d}"/>' + lab


def ground(x, y):
    return (
        f'<polyline points="{x},{y} {x},{y+12}"/>'
        f'<polyline points="{x-14},{y+12} {x+14},{y+12}"/>'
        f'<polyline points="{x-9},{y+18} {x+9},{y+18}"/>'
        f'<polyline points="{x-4},{y+24} {x+4},{y+24}"/>'
    )


def node(x, y, name):
    return (
        f'<circle cx="{x}" cy="{y}" r="3.5" fill="{ACC}" stroke="{ACC}"/>'
        f'<text x="{x}" y="{y-12}" text-anchor="middle" stroke="none" fill="{ACC}">{name}</text>'
    )


def emit(path, w, h, body):
    pathlib.Path(path).write_text(
        HEAD.format(w=w, h=h) + STYLE + body + "</g></svg>", encoding="utf-8"
    )
    print("wrote", path)


out = pathlib.Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)

# 1. source + resistor
emit(
    out / "circuit-source-resistor.svg",
    320,
    190,
    source(70, 100, "100 V")
    + wire((70, 82), (70, 50), (230, 50), (230, 78))
    + resistor(230, 100, "R = 10 Ω", vertical=True)
    + wire((230, 122), (230, 150), (70, 150), (70, 118))
    + ground(70, 150)
    + node(230, 50, "n1"),
)

# 2. source + R + L
emit(
    out / "circuit-rl.svg",
    380,
    190,
    source(70, 100, "100 V")
    + wire((70, 82), (70, 50), (148, 50))
    + resistor(170, 50, "R = 10 Ω")
    + wire((192, 50), (300, 50), (300, 76))
    + inductor(300, 100, "L = 50 mH")
    + wire((300, 124), (300, 150), (70, 150), (70, 118))
    + ground(70, 150)
    + node(70, 50, "n1")
    + node(300, 50, "n2"),
)

# 3. two-bus network: slack, line, load
emit(
    out / "circuit-two-bus.svg",
    430,
    200,
    source(60, 105, "slack")
    + wire((60, 87), (60, 55), (140, 55))
    + resistor(165, 55, "line")
    + wire((190, 55), (330, 55), (330, 80))
    + resistor(330, 105, "load", vertical=True)
    + wire((330, 127), (330, 160), (60, 160), (60, 123))
    + ground(60, 160)
    + node(60, 55, "n1")
    + node(330, 55, "n2"),
)

# 4. the same with a fault switch at the load bus
emit(
    out / "circuit-fault.svg",
    430,
    210,
    source(60, 105, "slack")
    + wire((60, 87), (60, 55), (140, 55))
    + resistor(165, 55, "line")
    + wire((190, 55), (330, 55))
    + wire((330, 55), (330, 80))
    + resistor(330, 105, "load", vertical=True)
    + wire((330, 127), (330, 165), (60, 165), (60, 123))
    + ground(60, 165)
    +
    # fault branch: switch to ground at n2
    wire((330, 55), (405, 55), (405, 80))
    + f'<polyline points="405,80 405,92"/><polyline points="405,118 405,130"/>'
    f'<polyline points="405,92 419,116"/>'
    f'<circle cx="405" cy="92" r="2.5" fill="{INK}"/><circle cx="405" cy="118" r="2.5" fill="{INK}"/>'
    f'<text x="425" y="109" stroke="none" fill="{INK}">fault</text>'
    + ground(405, 130)
    + node(60, 55, "n1")
    + node(330, 55, "n2"),
)


# The architecture diagrams under Developer Guide are NOT generated here. They are editable
# draw.io SVGs with the mxfile source embedded in the svg content attribute, drawn as UML class
# diagrams with orthogonal connectors and hollow inheritance arrowheads. Regenerating them from
# this script produced something that did not match the two diagrams beside it, twice. Edit them in
# draw.io instead; see item 6g in plans/DOCS_COMPLETION_PLAN.md for what is wrong with them.
