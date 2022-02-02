import villas.dataprocessing.readtools as rt
from villas.dataprocessing.timeseries import TimeSeries as ts
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Read simulation results
def results(sim_name, system, mpc_objects):

    work_dir = 'logs/' + sim_name +'/'
    log_name = sim_name
    print(work_dir + log_name + '.csv')

    ts_dpsimpy = rt.read_timeseries_dpsim(work_dir + log_name + '.csv')

    # ToDo: display results the same as Matpower 
    #Bus      Voltage          Generation             Load        
    #   Mag(pu) Ang(deg)   P (MW)   Q (MVAr)   P (MW)   Q (MVAr)

    results=pd.DataFrame()
    results = pd.DataFrame(columns=['Bus', 'Voltage Mag(pu)', 'Voltage Ang(deg)', 'P(MW)', 'Q (MVAr)'])

    for i in range(len(system.nodes)):
        node = system.nodes[i].name()
        node_baseV= mpc_objects.mpc_bus_data.loc[mpc_objects.mpc_bus_data['bus_i'] == int(node), 'baseKV'].iloc[0]*1e3
        w_mw= 1e-6

        results.loc[i] = [node] + ["{0:.6f}".format(np.absolute(ts_dpsimpy[node + '.V'].values[-1])/node_baseV)] + ["{0:.6f}".format(np.degrees(np.angle(ts_dpsimpy[node + '.V'].values[-1])))] + ["{0:.6f}".format(w_mw*np.real(ts_dpsimpy[node + '.S'].values[-1]))] + ["{0:.6f}".format(w_mw*np.imag(ts_dpsimpy[node + '.S'].values[-1]))]
    
    print(results)
    return results     
