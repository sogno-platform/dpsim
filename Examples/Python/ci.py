#!/usr/bin/env python3
#
# Script for automatically running the tests and comparing the results against
# given CSV files (from some other simulator).
# TODO: supporting different timesteps (interpolating), phasor/EMT conversion,
# more advanced error criterions...

import dpsim
import dpsim.components
import dpsim.components.dp

import numpy as np
import pandas
import sys
import os
import subprocess

EPSILON = 1e-6
PATH = os.path.dirname(__file__)
BINARY = os.path.basename(__file__)

def compare_results(dp_csv, expected_csv):
    dp_data = pandas.read_csv(dp_csv)
    expected_data = pandas.read_csv(expected_csv)
    if dp_data.shape[1] != expected_data.shape[1]:
        print("{}: result vector dimension mismatch (DP: {}, expected: {})".format(BINARY,
            dp_data.shape[1], expected_data.shape[1]), file=sys.stderr)
        return 1

    dp_time = np.array(dp_data.ix[:,0])
    expected_time = np.array(expected_data.ix[:,0])
    diff_time = dp_time - expected_time
    if np.any(diff_time):
        print("{}: time mismatch (wrong timestep?)".format(BINARY), file=sys.stderr)
        return 1

    ret = 0
    for i in range(1, int((dp_data.shape[1] - 1) / 2)):
        real_idx = i
        imag_idx = i + int((dp_data.shape[1] - 1) / 2)

        dp_real = np.array(dp_data.ix[:,real_idx])
        dp_imag = np.array(dp_data.ix[:,imag_idx])

        expected_real = np.array(expected_data.ix[:,real_idx])
        expected_imag = np.array(expected_data.ix[:,imag_idx])

        diff = np.sqrt((dp_real-expected_real)**2+(dp_imag-expected_imag)**2)
        diff_idx = np.nonzero(diff > EPSILON)

        if len(diff_idx[0]) != 0:
            print("{}: node {} has {} values above diff threshold".format(BINARY,
                i, len(diff_idx[0])), file=sys.stderr)
            print("(first at {} with diff of {})".format(diff_idx[0][0],
                diff[diff_idx[0][0]]), file=sys.stderr)
            ret = 1

    return ret

def run_python_test(name, sim):
    sim.run()

    dp_csv       = PATH + '/' + name + ".csv"
    expected_csv = PATH + '/' + name + ".expected.csv"

    return compare_results(dp_csv, expected_csv)

def run_cpp_test(name):
    args = PATH + '/../../build/Examples/Cxx/' + name
    popen = subprocess.call(args)

    #dp_csv       = PATH + '/' + name + ".csv"
    #expected_csv = PATH + '/' + name + ".expected.csv"

    return 0

if __name__ == "__main__":
    python_sims = {
        'TestSimple': dpsim.Simulation([
            dpsim.components.VoltSourceRes("v_s", 1, 0, 10000+0j, 1),
            dpsim.components.dp.Resistor("r_line", 1, 2, 1),
            dpsim.components.dp.Inductor("l_line", 2, 3, 1),
            dpsim.components.dp.Resistor("r_load", 3, 0, 1000)],
            duration=0.3, llog=PATH + "/TestSimple.csv")
    }

    cpp_sims = {
        'DP_IdealVS_PILine1',
        'DP_IdealVS_R1',
        'DP_IdealVS_RLC1',
        'DP_IdealVS_RXLine1',
        'DP_IdealVS_RXLine2',
        'DP_ResVS_PILine1',
        'DP_ResVS_RL1',
        'DP_ResVS_RLC1',
        'DP_ResVS_RXLine1',
        'EMT_IdealVS_R1',
        'EMT_ResVS_RXLine_Switch1'
    }

    ret = 0
    # test python examples
    for name, sim in python_sims.items():
        if run_python_test(name, sim):
            print("{} failed".format(name), file=sys.stderr)
            ret = 1
        else:
            print("{} successful".format(name), file=sys.stderr)

    # test cpp examples
    for name in cpp_sims:
        if run_cpp_test(name):
            print("{} failed".format(name), file=sys.stderr)
            ret = 1
        else:
            print("{} successful".format(name), file=sys.stderr)

    print("All tests successful.", file=sys.stderr)

    sys.exit(ret)
