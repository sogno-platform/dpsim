% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);

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
for dt = 0.00005: 0.00005: 0.003
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
    s = round(l_Ref/l);
    l_new=round(1/3*l_Ref);
    
    if l == l_Ref    
    CurrentVector_F = -currentShiftDP(l_new:2*l_new,2);
    else
    CurrentVector_interpolated = interp(-currentShiftDP(:,2),s);
    CurrentVector_F = CurrentVector_interpolated(l_new:2*l_new,:);
    end
    
    ReferenceCurrent = Results_Reference(:,5);
    ReferenceCurrent_F = ReferenceCurrent(l_new:2*l_new,:);
    
    
    Dif_F = abs(CurrentVector_F - ReferenceCurrent_F);
    [MaxDif_F,i1] = max(Dif_F);
    err_F = sqrt(immse(CurrentVector_F,ReferenceCurrent_F));
    RMS_ref_F = rms(ReferenceCurrent_F);
    %%disp(['RMS ia steady state: ', num2str(RMS_ref_SteadyState), ' A', '(dt = ', num2str(dt), ')' ]);
    %%disp(['Maximum Error ia steady state: ', num2str(MaxDif_SS), ' A']);
    %%disp(['Root Mean-squared error ia steady state: ', num2str(err_SS), ' A'] );
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['FAULT:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_F/(RMS_ref_F*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_F/(RMS_ref_F*sqrt(2))), ' %']);

    ErrorVector_F(i) = 100*err_F/(RMS_ref_F*sqrt(2));
    dtVector(i) = dt;
    i = i + 1;
end

title('Current Phase a - VBR');
 
 figure(2)
 hold off
 plot2 = plot(dtVector,ErrorVector_F);
 hold on 
 %% Dq Model
 
figure(3)
hold off
plot(Results_Reference(:,1),Results_Reference(:,5))
hold on

i = 1;

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

%read results from c++ simulation
for dt = 0.00005: 0.00005: 0.0006
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
    
     l=length(currentDP);
    s = round(l_Ref/l);
    l_new=round(1/3*l_Ref);
    
    if l == l_Ref    
    CurrentVector_F = currentShiftDP(l_new:2*l_new,2);
    else
    CurrentVector_interpolated = interp(currentShiftDP(:,2),s);
    CurrentVector_F = CurrentVector_interpolated(l_new:2*l_new,:);
    end
    
    ReferenceCurrent = Results_Reference(:,5);
    ReferenceCurrent_F = ReferenceCurrent(l_new:2*l_new,:);
    
    
    Dif_F = abs(CurrentVector_F - ReferenceCurrent_F);
    [MaxDif_F,i1] = max(Dif_F);
    err_F = sqrt(immse(CurrentVector_F,ReferenceCurrent_F));
    RMS_ref_F = rms(ReferenceCurrent_F);
    %%disp(['RMS ia steady state: ', num2str(RMS_ref_SteadyState), ' A', '(dt = ', num2str(dt), ')' ]);
    %%disp(['Maximum Error ia steady state: ', num2str(MaxDif_SS), ' A']);
    %%disp(['Root Mean-squared error ia steady state: ', num2str(err_SS), ' A'] );
    disp(['##################### (dt = ', num2str(dt), ') ################################'])
    disp(['FAULT:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_F/(RMS_ref_F*sqrt(2))), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_F/(RMS_ref_F*sqrt(2))), ' %']);

    ErrorVector_F2(i) = 100*err_F/(RMS_ref_F*sqrt(2));
    dtVector2(i) = dt;
    i = i + 1;
end
title('Current Phase a - DQ');

 figure(2)
  plot4 = plot(dtVector2,ErrorVector_F2);
 legend('Load Change Error _ VBR','Load Change Error _ DQ')
 ylabel('Error [%]');
 xlabel('Time Step [s]');
 
 set(plot2,'LineWidth',2);
 set(plot4,'LineWidth',2);