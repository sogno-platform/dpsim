# Compare voltage and current of c++ simulation with voltage and currents
# from PLECS simulation

from dataprocessing.plotdpsim import *

path = 'C:\\Users\\mmi\\git\\PowerSystemSimulation\\vsa\\Results\\'

# read PLECS results
results_plecs = read_timeseries_PLECS(path + 'SynGenDqEmt_ABCFault_PLECS\\Voltages_and_currents.csv')

# read DPsim results
results_dpsim = read_timeseries_DPsim(path + 'SynGenDqEmt_ABCFault_DPsim_1_Damping\\data_j.csv')

# Stator currents
figure_id = 1
plt.figure(figure_id)
plt.title("Stator currents")
#set_time_series_labels(Results_PLECS, ['PLECS V1', 'PLECS V2', 'PLECS V3', 'PLECS I1', 'PLECS I2', 'PLECS I3'])
plot_timeseries(figure_id, results_dpsim, plt_linestyle='-')
plot_timeseries(figure_id, [results_plecs[3], results_plecs[4], results_plecs[5]], plt_linestyle='--')
plt.xlabel('Time [s]')
plt.show()