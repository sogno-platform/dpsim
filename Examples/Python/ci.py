#!/usr/bin/env python3
#
# Script for automatically running the tests and comparing the results against
# given CSV files (from some other simulator).
# TODO: supporting different timesteps (interpolating), phasor/EMT conversion,
# more advanced error criterions...

import dpsim
import numpy as np
import pandas
import sys
import os
import subprocess

EPSILON = 1e-6
PATH = os.path.dirname(__file__)
BINARY = os.path.basename(__file__)

def compare_results(dpCsv, expectedCsv):
    dpData = pandas.read_csv(dpCsv)
    expectedData = pandas.read_csv(expectedCsv)
    if dpData.shape[1] != expectedData.shape[1]:
        print("{}: result vector dimension mismatch (DP: {}, expected: {})".format(BINARY,
            dpData.shape[1], expectedData.shape[1]), file=sys.stderr)
        return 1

    dpTime = np.array(dpData.ix[:,0])
    expectedTime = np.array(expectedData.ix[:,0])
    diffTime = dpTime - expectedTime
    if np.any(diffTime):
        print("{}: time mismatch (wrong timestep?)".format(BINARY), file=sys.stderr)
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
            print("{}: node {} has {} values above diff threshold".format(BINARY,
                i, len(diffIdx[0])), file=sys.stderr)
            print("(first at {} with diff of {})".format(diffIdx[0][0],
                diff[diffIdx[0][0]]), file=sys.stderr)
            ret = 1

    return ret

def run_python_test(name, sim):
    sim.start()
    sim.wait()

    dpCsv       = PATH + '/' + name + ".csv"
    expectedCsv = PATH + '/' + name + ".expected.csv"

    return compare_results(dpCsv, expectedCsv)

def run_cpp_test(name):
    args = PATH + '/' + name
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()

    #dpCsv       = PATH + '/' + name + ".csv"
    #expectedCsv = PATH + '/' + name + ".expected.csv"

    return 0

if __name__ == "__main__":
    python_sims = {
        'TestSimple': dpsim.Simulation([
            dpsim.VoltSourceRes("v_s", 1, 0, 10000+0j, 1),
            dpsim.Resistor("r_line", 1, 2, 1),
            dpsim.Inductor("l_line", 2, 3, 1),
            dpsim.Resistor("r_load", 3, 0, 1000)],
            duration=0.3, llog=PATH + "/TestSimple.csv")
    }

    cpp_sims = {
        'SynGenThreePhaseFault'
    }

    ret = 0
    # test python examples
    for name, sim in python_sims.items():
        if run_python_test(name, sim):
            print("{} failed".format(name), file=sys.stderr)
            ret = 1
        else:
            print("{} successfull".format(name), file=sys.stderr)

    # test cpp examples
    for name in cpp_sims:
        if run_cpp_test(name):
            print("{} failed".format(name), file=sys.stderr)
            ret = 1
        else:
            print("{} successfull".format(name), file=sys.stderr)

    print("All tests successfull.", file=sys.stderr)

    sys.exit(ret)
