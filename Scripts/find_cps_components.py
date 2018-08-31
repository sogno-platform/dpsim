#!/bin/env python

import sys
import glob
import re

if len(sys.argv) not in [2, 3]:
    print("usage: %s DIR [OUTPUTFILE]" % sys.argv[0])
    sys.exit(-1)

directory = sys.argv[1]

filenames = glob.glob(directory + "/**/*.h", recursive = True)

regex =  r"namespace\s+(CPS)\s+\{[\W\w]+" \
         r"namespace\s+(EMT|DP)\s+\{[\W\w]+" \
         r"namespace\s+(Ph\d+)\s+\{[\W\w]+" \
         r"class\s+(\w+)"

classnames = [ ]

for filename in filenames:
    with open(filename) as f:
        contents = f.read()

        for match in re.finditer(regex, contents, re.MULTILINE):
            classname = "Component::getConstructorDef<{}::{}::{}::{}>(\"_{}_{}_{}\"),\n".format(
                match[1], match[2], match[3], match[4],
                match[2].lower(), match[3].lower(), match[4]
            )

            classnames.append(classname)


blacklist = [
    "CPS::DP::Ph3::SynchronGeneratorDQSmpl",
    "CPS::DP::Ph3::SynchronGeneratorVBR",
    "CPS::DP::Ph3::SynchronGeneratorVBRStandalone",
    "CPS::EMT::Ph3::SynchronGeneratorDQ",
    "CPS::EMT::Ph3::SynchronGeneratorDQSmpl",
    "CPS::EMT::Ph3::SynchronGeneratorDQSmplCompSource",
    "CPS::EMT::Ph3::SynchronGeneratorVBR",
    "CPS::EMT::Ph3::SynchronGeneratorVBRSmpl",
    "CPS::EMT::Ph3::SynchronGeneratorVBRStandalone"
]

filtered_classnames = [ x for x in classnames if not any(y in x for y in blacklist) ]
sorted_classnames = sorted(filtered_classnames)

if len(sys.argv) == 3:
    output_file = open(sys.argv[2], 'w')
else:
    output_file = sys.stdout

output_file.writelines(sorted_classnames)
