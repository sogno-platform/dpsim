% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/ABCFault/Simulink/Voltages_and_currents.csv');
%l_Ref = length(Results_Reference);

%% read results from c++ simulation
for dt = 0.0001: 0.00005: 0.001
    FileName = strcat('SynGen_VBR_', num2str(dt),'.csv');
    Path = strcat('../../../vsa/Results/ABCFault/DPsim/VBR/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:4);
    
    l=length(CurrentVector);
    l_new=round(1/3*l);
    CurrentVector_SteadyState = -CurrentVector(1:l_new,:);
    
    ReferenceCurrent = Results_Reference(:,5);
    ReferenceCurrent_resampled = resample(ReferenceCurrent,l,length(ReferenceCurrent));
    ReferenceCurrent_resampled_SS = ReferenceCurrent_resampled(1:l_new,:);
    
    Dif_SS = abs(CurrentVector_SteadyState(:,2) - ReferenceCurrent_resampled_SS);
    [MaxDif_SS,i1] = max(Dif_SS);
    err_SS = sqrt(immse(CurrentVector_SteadyState(:,2),ReferenceCurrent_resampled_SS));
    RMS_ref_SteadyState = rms(ReferenceCurrent_resampled_SS);
    
    %%disp(['RMS ia steady state: ', num2str(RMS_ref_SteadyState), ' A', '(dt = ', num2str(dt), ')' ]);
    %%disp(['Maximum Error ia steady state: ', num2str(MaxDif_SS), ' A']);
    %%disp(['Root Mean-squared error ia steady state: ', num2str(err_SS), ' A'] );
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);

end




