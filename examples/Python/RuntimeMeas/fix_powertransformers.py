#!/usr/bin/python3

import sys
import xml.etree.ElementTree

if len(sys.argv) != 3:
    sys.exit("usage: fix_powertransformers.py <old.xml> <new.xml>")

tree = xml.etree.ElementTree.parse(sys.argv[1])

ns = {
    "cim": "http://iec.ch/TC57/2009/CIM-schema-cim14#",
    "rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
}

pt_ends = dict()
for pt_end in tree.iterfind("cim:PowerTransformerEnd", namespaces=ns):
    pt_ends[pt_end.get("{" + ns["rdf"] + "}ID")] = pt_end

for terminal in tree.iterfind("cim:Terminal", ns):
    condeq = terminal.find("cim:Terminal.ConductingEquipment", ns)
    condeq_id = condeq.get("{" + ns["rdf"] + "}resource")[1:]
    if condeq_id in pt_ends:
        pt_end = pt_ends[condeq_id]
        pt_id = pt_end.find(
            "cim:PowerTransformerEnd.MemberOf_PowerTransformer", ns
        ).get("{" + ns["rdf"] + "}resource")[1:]
        terminal_id = terminal.get("{" + ns["rdf"] + "}ID")
        end_term = xml.etree.ElementTree.Element(
            "cim:TransformerEnd.Terminal", {"rdf:resource": "#" + terminal_id}
        )
        pt_end.append(end_term)
        condeq.set("{" + ns["rdf"] + "}resource", "#" + pt_id)

tree.write(sys.argv[2])
