#!/usr/bin/python3

import csv
import sys

from meas_utils import *

if len(sys.argv) != 4:
    sys.exit("usage: speedup.py old.csv new.csv outname")

old = Measurement.read_csv(sys.argv[1])
new = Measurement.read_csv(sys.argv[2])

meas = new.speedup(old, sys.argv[3])
meas.save()
