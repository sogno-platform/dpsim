% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);
l_new = l_Ref/3;
Current_Ref_LC = Results_Reference(l_new:2*l_new,5);
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
    'SynGen_VBR_0.001000.csv';
    'SynGen_VBR_0.001050.csv';
    'SynGen_VBR_0.001100.csv';
    'SynGen_VBR_0.001150.csv';
    'SynGen_VBR_0.001200.csv';
    'SynGen_VBR_0.001250.csv';
    'SynGen_VBR_0.001300.csv';
    'SynGen_VBR_0.001350.csv';
    'SynGen_VBR_0.001400.csv';
    'SynGen_VBR_0.001450.csv';
    'SynGen_VBR_0.001500.csv';
    'SynGen_VBR_0.001550.csv';
    'SynGen_VBR_0.001600.csv';
    'SynGen_VBR_0.001650.csv';
    'SynGen_VBR_0.001700.csv';
    'SynGen_VBR_0.001750.csv';
    'SynGen_VBR_0.001800.csv';
    'SynGen_VBR_0.001850.csv';
    'SynGen_VBR_0.001900.csv';
    'SynGen_VBR_0.001950.csv';
    'SynGen_VBR_0.002000.csv';
    'SynGen_VBR_0.002050.csv';
    'SynGen_VBR_0.002100.csv';
    'SynGen_VBR_0.002150.csv';
    'SynGen_VBR_0.002200.csv';
    'SynGen_VBR_0.002250.csv';
    'SynGen_VBR_0.002300.csv';
    'SynGen_VBR_0.002350.csv';
    'SynGen_VBR_0.002400.csv';
    'SynGen_VBR_0.002450.csv';
    'SynGen_VBR_0.002500.csv';
    'SynGen_VBR_0.002550.csv';
    'SynGen_VBR_0.002600.csv';
    'SynGen_VBR_0.002650.csv';
    'SynGen_VBR_0.002700.csv';
    'SynGen_VBR_0.002750.csv';
    'SynGen_VBR_0.002800.csv';
    'SynGen_VBR_0.002850.csv';
    'SynGen_VBR_0.002900.csv';
    'SynGen_VBR_0.002950.csv';
    'SynGen_VBR_0.003000.csv'];

%% read results from c++ simulation
for dt = 0.000050: 0.000050: 0.003
    FileName = FileNameVector(i,1:23);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/EMT/VBR/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:2);
    
  
    l=length(CurrentVector);
    s = round(l_Ref/l);
    CurrentVector_interpolated = interp(CurrentVector(:,2),s);
    Time_interpolated = interp(CurrentVector(:,1),s);
    CurrentVector_interpolated_LC = -CurrentVector_interpolated(l_new:2*l_new);
    plot(Time_interpolated,-CurrentVector_interpolated);
    

    
    
    Dif_LC = abs(Current_Ref_LC - CurrentVector_interpolated_LC);
    [MaxDif_LC,i1] = max(Dif_LC);
    err_LC = sqrt(immse(Current_Ref_LC,CurrentVector_interpolated_LC));
    Peak_Ref = rms(Current_Ref_LC)*sqrt(2);
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['AFTER LOAD CHANGE:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_LC/Peak_Ref), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_LC/Peak_Ref), ' %']);

    ErrorVector_LC(i) = 100*err_LC/Peak_Ref;
    dtVector(i) = dt;
    i = i + 1;
end

 
 figure(2)
 hold off
plotF1 = plot(dtVector,ErrorVector_LC);
 hold on
 
 figure(3)
hold off
plot(Results_Reference(l_new:2*l_new,1),Current_Ref_LC)
hold on
FileNameVector = ['SynGen_Dq_0.000050.csv';
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
j = 1
%% read results from c++ simulation
for dt = 0.000050: 0.000050: 0.0006
    FileName = FileNameVector(j,1:22);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/EMT/Dq/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:4);
    
  
    l=length(CurrentVector);
    s = round(l_Ref/l);
    CurrentVector_interpolated = interp(CurrentVector(:,2),s);
    Time_interpolated = interp(CurrentVector(:,1),s);
    CurrentVector_interpolated_LC = CurrentVector_interpolated(l_new:2*l_new);
    plot(Time_interpolated,CurrentVector_interpolated);
    

    
    
    Dif_LC = abs(Current_Ref_LC - CurrentVector_interpolated_LC);
    [MaxDif_LC,i1] = max(Dif_LC);
    err_LC = sqrt(immse(Current_Ref_LC,CurrentVector_interpolated_LC));
    Peak_Ref = rms(Current_Ref_LC)*sqrt(2);
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['AFTER LOAD CHANGE:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_LC/Peak_Ref), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_LC/Peak_Ref), ' %']);

    ErrorVector_LC2(j) = 100*err_LC/Peak_Ref;
    dtVector2(j) = dt;
    j = j + 1;
end
 
 figure(2)
plotF2 = plot(dtVector2,ErrorVector_LC2);
 hold on
 set(plotF2,'LineWidth',2);
 set(plotF1,'LineWidth',2);
 legend('Error VBR', 'Error Classical')
 xlabel('Time Step [s]');
 ylabel('Root mean squared error [%]')
