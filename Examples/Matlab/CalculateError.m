% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);
Results_Reference = Results_Reference(1:6000,:);
figure(1)
hold off
plot(Results_Reference(:,1),Results_Reference(:,5))
hold on

i = 1;

FileNameVector = ['SynGen_VBR_0.000050.csv';
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
for dt = 0.000050: 0.000050: 0.001
    FileName = FileNameVector(i,1:23);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/EMT/VBR/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:4);
    
 
    
    
    l=length(CurrentVector);
    l_new=round(1/3*l);
    CurrentVector_LC = CurrentVector(l_new+3:2*l_new-1,:);
    
    ReferenceCurrent = Results_Reference(:,5);
    ReferenceCurrent_resampled = resample(ReferenceCurrent,l,length(ReferenceCurrent));
    ReferenceCurrent_resampled_LC = ReferenceCurrent_resampled(l_new+3:2*l_new-1,:);

     figure(3)
    plot(CurrentVector_LC(:,1),ReferenceCurrent_resampled_LC);
    hold on
    
    Dif_LC = abs(-CurrentVector_LC(:,2) - ReferenceCurrent_resampled_LC);
    [MaxDif_LC,i1] = max(Dif_LC);
    err_LC = sqrt(immse(-CurrentVector_LC(:,2),ReferenceCurrent_resampled_LC));
    RMS_ref_F = rms(ReferenceCurrent_resampled_LC);
   disp(['##################### (dt = ', num2str(dt), ') ################################'])
   disp(['AFTER LOAD CHANGE:'])
   disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_LC/(RMS_ref_F*sqrt(2))), ' %']);
   disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_LC/(RMS_ref_F*sqrt(2))), ' %']);

    ErrorVector_LC(i) = 100*err_LC/(RMS_ref_F*sqrt(2));
    dtVector(i) = dt;
    i = i + 1;
end
 
 figure(2)
 hold off
plotF1 = plot(dtVector,ErrorVector_LC);
 hold on
 
 
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
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/EMT/Dq/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:4);
    
    plot(CurrentVector(:,1),CurrentVector(:,2))
    
    l=length(CurrentVector);
    l_new=round(1/3*l);
    CurrentVector_SteadyState = CurrentVector(1:l_new,:);
    CurrentVector_LC = CurrentVector(l_new+1:2*l_new-1,:);
    
    ReferenceCurrent = Results_Reference(:,5);
    ReferenceCurrent_resampled = resample(ReferenceCurrent,l,length(ReferenceCurrent));
    ReferenceCurrent_resampled_SS = ReferenceCurrent_resampled(1:l_new,:);
    ReferenceCurrent_resampled_LC = ReferenceCurrent_resampled(l_new+1:2*l_new-1,:);
    
    Dif_SS = abs(CurrentVector_SteadyState(:,2) - ReferenceCurrent_resampled_SS);
    [MaxDif_SS,i1] = max(Dif_SS);
    err_SS = sqrt(immse(CurrentVector_SteadyState(:,2),ReferenceCurrent_resampled_SS));
    RMS_ref_SteadyState = rms(ReferenceCurrent_resampled_SS);
    
    
    Dif_LC = abs(CurrentVector_LC(:,2) - ReferenceCurrent_resampled_LC);
    [MaxDif_LC,i1] = max(Dif_LC);
    err_LC = sqrt(immse(CurrentVector_LC(:,2),ReferenceCurrent_resampled_LC));
    RMS_ref_F = rms(ReferenceCurrent_resampled_LC);

    disp(['   '])
    disp(['##################### Results for DQ Model ################################'])
    disp(['   '])
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['STEADY STATE:'])
    disp(['  Maximum Error ia steady state: ', num2str(100*MaxDif_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia steady state: ', num2str(100*err_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
    disp(['FAULT:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_LC/(RMS_ref_F*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_LC/(RMS_ref_F*sqrt(2))), ' %']);

    ErrorVector_SS2(i) = 100*err_SS/(RMS_ref_SteadyState*sqrt(2));
    ErrorVector_F2(i) = 100*err_LC/(RMS_ref_F*sqrt(2));
    dtVector2(i) = dt;
    i = i + 1;
end

 figure(2)
 plotSS2 = plot(dtVector2,ErrorVector_SS2);
 plotF2 = plot(dtVector2,ErrorVector_F2);
 legend('Steady state error - VBR','Load Change Error - VBR','Steady state error _ DQ','Load Change Error _ DQ')
 ylabel('Error');
 xlabel('Time Step');
 title('Error in function of time step')
 set(plotSS2,'LineWidth',2);
 set(plotF2,'LineWidth',2);
 set(plotSS1,'LineWidth',2);
 set(plotF1,'LineWidth',2);