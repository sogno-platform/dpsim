#!/usr/bin/python3
#
# Script for automatically running the tests and comparing the results against
# given CSV files (from some other simulator).
# TODO: supporting different timesteps (interpolating), phasor/EMT conversion,
# more advanced error criterions...

import glob
import numpy as np
import pandas
import subprocess
import sys

EPSILON = 1e-6

def run_test(binary, dpCsv, expectedCsv):
    ret = subprocess.call("./" + binary)
    if ret:
        print(binary + " binary returned code " + str(ret), file=sys.stderr)
        return ret
    dpData = pandas.read_csv(dpCsv, header=None)
    expectedData = pandas.read_csv(expectedCsv, header=None)
    if dpData.shape[1] != expectedData.shape[1]:
        print("{}: result vector dimension mismatch (DP: {}, expected: {}".format(binary,
            dpData.shape[1], expectedData.shape[1]), file=sys.stderr)
        return 1

    dpTime = np.array(dpData.ix[:,0])
    expectedTime = np.array(expectedData.ix[:,0])
    diffTime = dpTime - expectedTime
    if np.any(diffTime):
        print(binary + ": time mismatch (wrong timestep?)")
        return 1

    ret = 0
    for i in range(1, int((dpData.shape[1] - 1) / 2)):
        realIdx = i
        imagIdx = i + int((dpData.shape[1] - 1) / 2)
        dpReal = np.array(dpData.ix[:,realIdx])
        dpImag = np.array(dpData.ix[:,imagIdx])
        expectedReal = np.array(expectedData.ix[:,realIdx])
        expectedImag = np.array(expectedData.ix[:,imagIdx])
        diff = np.sqrt((dpReal-expectedReal)**2+(dpImag-expectedImag)**2)
        diffIdx = np.nonzero(diff > EPSILON)
        if len(diffIdx[0]) != 0:
            print("{}: node {} has {} values above diff threshold".format(binary,
                i, len(diffIdx[0])), file=sys.stderr)
            print("(first at {} with diff of {})".format(diffIdx[0][0],
                diff[diffIdx[0][0]]), file=sys.stderr)
            ret = 1
    return ret


if __name__ == "__main__":
    sources = glob.glob("Test*.cpp")
    bins = ["../build/" + s.replace(".cpp", "") for s in sources]
    dpCsvs = [s.replace(".cpp", ".csv") for s in sources]
    expectedCsvs = [s.replace(".cpp", ".expected.csv") for s in sources]
    ret = 0
    for i in range(0, len(sources)):
        if run_test(bins[i], dpCsvs[i], expectedCsvs[i]):
            print(bins[i] + " failed!", file=sys.stderr)
            ret = 1
        else:
            print(bins[i] + " successfull.")
    if not ret:
        print("All tests successfull.")
    sys.exit(ret)
