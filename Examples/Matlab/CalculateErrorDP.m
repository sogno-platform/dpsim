% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
%l_Ref = length(Results_Reference);

figure(1)
hold off
plot(Results_Reference(:,1),Results_Reference(:,5))
hold on

i = 1;

FileNameVector = ['SynGen_VBR_0.000050.csv';
    'SynGen_VBR_0.000050.csv';
    'SynGen_VBR_0.000100.csv';
    'SynGen_VBR_0.000150.csv';
    'SynGen_VBR_0.000200.csv';
    'SynGen_VBR_0.000250.csv';
    'SynGen_VBR_0.000300.csv';
    'SynGen_VBR_0.000350.csv';
    'SynGen_VBR_0.000400.csv';
    'SynGen_VBR_0.000450.csv';
    'SynGen_VBR_0.000500.csv';
    'SynGen_VBR_0.000550.csv';
    'SynGen_VBR_0.000600.csv';
    'SynGen_VBR_0.000650.csv';
    'SynGen_VBR_0.000700.csv';
    'SynGen_VBR_0.000750.csv';
    'SynGen_VBR_0.000800.csv';
    'SynGen_VBR_0.000850.csv';
    'SynGen_VBR_0.000900.csv';
    'SynGen_VBR_0.000950.csv';
    'SynGen_VBR_0.001000.csv'];

%% read results from c++ simulation
for dt = 0.00005: 0.0001: 0.001
    FileName = FileNameVector(i,1:23);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/DP/VBR/', FileName);
    Log_SynGen = csvread(Path,1);
    currentDP = Log_SynGen(:,1:7);
    compOffsetDP = (size(currentDP,2) - 1) / 2;
    
    currentShiftDP = currentDP(:,1);
    col = 2;
    for row = 1:size(currentDP,1)
        currentShiftDP(row,col) = currentDP(row,col)*cos(2*pi*60*currentDP(row,1)) - ...
            currentDP(row,col+compOffsetDP)*sin(2*pi*60*currentDP(row,1));
    end
    plot(currentShiftDP(:,1),-currentShiftDP(:,2))
    
    
    l=length(currentDP);
    l_new=round(1/3*l);
    CurrentVector_SteadyState = -currentShiftDP(1:l_new,:);
    CurrentVector_F = -currentShiftDP(l_new+1:2*l_new-1,:);
    
    ReferenceCurrent = Results_Reference(:,5);
    ReferenceCurrent_resampled = resample(ReferenceCurrent,l,length(ReferenceCurrent));
    ReferenceCurrent_resampled_SS = ReferenceCurrent_resampled(1:l_new,:);
    ReferenceCurrent_resampled_F = ReferenceCurrent_resampled(l_new+1:2*l_new-1,:);
    
    Dif_SS = abs(CurrentVector_SteadyState(:,2) - ReferenceCurrent_resampled_SS);
    [MaxDif_SS,i1] = max(Dif_SS);
    err_SS = sqrt(immse(CurrentVector_SteadyState(:,2),ReferenceCurrent_resampled_SS));
    RMS_ref_SteadyState = rms(ReferenceCurrent_resampled_SS);
    
    
    Dif_F = abs(CurrentVector_F(:,2) - ReferenceCurrent_resampled_F);
    [MaxDif_F,i1] = max(Dif_F);
    err_F = sqrt(immse(CurrentVector_F(:,2),ReferenceCurrent_resampled_F));
    RMS_ref_F = rms(ReferenceCurrent_resampled_F);
    %%disp(['RMS ia steady state: ', num2str(RMS_ref_SteadyState), ' A', '(dt = ', num2str(dt), ')' ]);
    %%disp(['Maximum Error ia steady state: ', num2str(MaxDif_SS), ' A']);
    %%disp(['Root Mean-squared error ia steady state: ', num2str(err_SS), ' A'] );
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['STEADY STATE:'])
    disp(['  Maximum Error ia steady state: ', num2str(100*MaxDif_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia steady state: ', num2str(100*err_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
    disp(['FAULT:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_F/(RMS_ref_F*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_F/(RMS_ref_F*sqrt(2))), ' %']);

    ErrorVector_SS(i) = 100*err_SS/(RMS_ref_SteadyState*sqrt(2));
    ErrorVector_F(i) = 100*err_F/(RMS_ref_F*sqrt(2));
    dtVector(i) = dt;
    i = i + 1;
end

title('Current Phase a - VBR');
 
 figure(2)
 hold off
 plot(dtVector,ErrorVector_SS);
 hold on
 plot(dtVector,ErrorVector_F);
 
 %% Dq Model
 
figure(3)
hold off
plot(Results_Reference(:,1),Results_Reference(:,5))
hold on

i = 1;

FileNameVector = ['SynGen_Dq_0.000050.csv';
    'SynGen_Dq_0.000050.csv';
    'SynGen_Dq_0.000100.csv';
    'SynGen_Dq_0.000150.csv';
    'SynGen_Dq_0.000200.csv';
    'SynGen_Dq_0.000250.csv';
    'SynGen_Dq_0.000300.csv';
    'SynGen_Dq_0.000350.csv';
    'SynGen_Dq_0.000400.csv';
    'SynGen_Dq_0.000450.csv';
    'SynGen_Dq_0.000500.csv';
    'SynGen_Dq_0.000550.csv';
    'SynGen_Dq_0.000600.csv';
    'SynGen_Dq_0.000650.csv';
    'SynGen_Dq_0.000700.csv'];

%read results from c++ simulation
for dt = 0.00005: 0.00005: 0.0007
    FileName = FileNameVector(i,1:22);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/DP/Dq/', FileName);
    Log_SynGen = csvread(Path,1);
    currentDP = Log_SynGen(:,1:7);
    compOffsetDP = (size(currentDP,2) - 1) / 2;
    
    currentShiftDP = currentDP(:,1);
    col = 2;
    for row = 1:size(currentDP,1)
        currentShiftDP(row,col) = currentDP(row,col)*cos(2*pi*60*currentDP(row,1)) - ...
            currentDP(row,col+compOffsetDP)*sin(2*pi*60*currentDP(row,1));
    end

    
    plot(currentDP(:,1),currentShiftDP(:,2))
    
    l=length(currentShiftDP);
    l_new=round(1/3*l);
    CurrentVector_SteadyState = currentShiftDP(1:l_new,:);
    CurrentVector_F = currentShiftDP(l_new+1:2*l_new-1,:);
    
    ReferenceCurrent = Results_Reference(:,5);
    ReferenceCurrent_resampled = resample(ReferenceCurrent,l,length(ReferenceCurrent));
    ReferenceCurrent_resampled_SS = ReferenceCurrent_resampled(1:l_new,:);
    ReferenceCurrent_resampled_F = ReferenceCurrent_resampled(l_new+1:2*l_new-1,:);
    
    Dif_SS = abs(CurrentVector_SteadyState(:,2) - ReferenceCurrent_resampled_SS);
    [MaxDif_SS,i1] = max(Dif_SS);
    err_SS = sqrt(immse(CurrentVector_SteadyState(:,2),ReferenceCurrent_resampled_SS));
    RMS_ref_SteadyState = rms(ReferenceCurrent_resampled_SS);
    
    
    Dif_F = abs(CurrentVector_F(:,2) - ReferenceCurrent_resampled_F);
    [MaxDif_F,i1] = max(Dif_F);
    err_F = sqrt(immse(CurrentVector_F(:,2),ReferenceCurrent_resampled_F));
    RMS_ref_F = rms(ReferenceCurrent_resampled_F);

    disp(['   '])
    disp(['##################### Results for DQ Model ################################'])
    disp(['   '])
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['STEADY STATE:'])
    disp(['  Maximum Error ia steady state: ', num2str(100*MaxDif_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia steady state: ', num2str(100*err_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
    disp(['FAULT:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_F/(RMS_ref_F*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_F/(RMS_ref_F*sqrt(2))), ' %']);

    ErrorVector_SS2(i) = 100*err_SS/(RMS_ref_SteadyState*sqrt(2));
    ErrorVector_F2(i) = 100*err_F/(RMS_ref_F*sqrt(2));
    dtVector2(i) = dt;
    i = i + 1;
end
title('Current Phase a - DQ');

 figure(2)
 plot(dtVector2,ErrorVector_SS2);
 plot(dtVector2,ErrorVector_F2);
 legend('Steady state error _ VBR','Load Change Error _ VBR','Steady state error _ DQ','Load Change Error _ DQ')
 ylabel('Error');
 xlabel('Time Step');