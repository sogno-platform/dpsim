% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Results_Reference(1:l_Ref/3,5));
Peak_Ref_LC = max(Results_Reference(l_Ref/3:2*l_Ref/3,5));

i = 1;

FileNameVector = ['SynGen_VBR_0.000050.csv';
    'SynGen_VBR_0.000100.csv';
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
for dt = 0.00005: 0.00005: 0.001
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

    
    l_DP=length(currentShiftDP);
    l_new_DP=round(1/3*l_DP);

    CurrentVector_SS_DP = -currentShiftDP(1:l_new_DP,2);
    CurrentVector_LC_DP = -currentShiftDP(l_new_DP:2*l_new_DP,2);

    CurrentReference_reduced = zeros(l_DP,2);
    
    if l_DP == l_Ref
        CurrentReference_reduced(:,1) = Results_Reference(:,1);
        CurrentReference_reduced(:,2) = Results_Reference(:,5);
    else
        s = round(dt/5e-5);
        n = 1;
        for m = 1:s:l_Ref
            CurrentReference_reduced(n,1) = Results_Reference(m,1);
            CurrentReference_reduced(n,2) = Results_Reference(m,5);
            n = n+1;
        end
    end  
 
    %Reference current in Steady state and after load change
    Reference_SS = CurrentReference_reduced(1:l_new_DP,2);
    Reference_LC = CurrentReference_reduced(l_new_DP:2*l_new_DP,2);

    % Calculate maximum error and root mean squared error for steady state
    Dif_SS_DP = abs(CurrentVector_SS_DP - Reference_SS);
    [MaxDif_SS_DP,i1] = max(Dif_SS_DP);
    err_SS_DP = sqrt(immse(CurrentVector_SS_DP,Reference_SS));
    disp('############ Error for VBR DP model ###############');
    disp(['############     dt= ',num2str(dt),'         ###############']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS_DP/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS_DP/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC_DP = abs(CurrentVector_LC_DP - Reference_LC);
    [MaxDif_LC_DP,i1] = max(Dif_LC_DP);
    err_LC_DP = sqrt(immse(CurrentVector_LC_DP,Reference_LC));
    disp(' ');
    disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC_DP/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC_DP/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');

    ErrorVector_LC(i) = 100*err_LC_DP/Peak_Ref_LC;
    dtVector(i) = dt;
    i = i + 1;
end


figure(2)
hold off
plot2 = plot(dtVector,ErrorVector_LC);
hold on 


%% Dq Model

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

    
    l_DP=length(currentShiftDP);
    l_new_DP=round(1/3*l_DP);

    CurrentVector_SS_DP = currentShiftDP(1:l_new_DP,2);
    CurrentVector_LC_DP = currentShiftDP(l_new_DP:2*l_new_DP,2);

    CurrentReference_reduced = zeros(l_DP,2);
    
    if l_DP == l_Ref
        CurrentReference_reduced(:,1) = Results_Reference(:,1);
        CurrentReference_reduced(:,2) = Results_Reference(:,5);
    else
        s = round(dt/5e-5);
        n = 1;
        for m = 1:s:l_Ref
            CurrentReference_reduced(n,1) = Results_Reference(m,1);
            CurrentReference_reduced(n,2) = Results_Reference(m,5);
            n = n+1;
        end
    end  
 
    %Reference current in Steady state and after load change
    Reference_SS = CurrentReference_reduced(1:l_new_DP,2);
    Reference_LC = CurrentReference_reduced(l_new_DP:2*l_new_DP,2);

    % Calculate maximum error and root mean squared error for steady state
    Dif_SS_DP = abs(CurrentVector_SS_DP - Reference_SS);
    [MaxDif_SS_DP,i1] = max(Dif_SS_DP);
    err_SS_DP = sqrt(immse(CurrentVector_SS_DP,Reference_SS));
    disp('############ Error for Dq DP model ###############');
    disp(['############     dt= ',num2str(dt),'         ###############']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS_DP/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS_DP/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC_DP = abs(CurrentVector_LC_DP - Reference_LC);
    [MaxDif_LC_DP,i1] = max(Dif_LC_DP);
    err_LC_DP = sqrt(immse(CurrentVector_LC_DP,Reference_LC));
    disp(' ');
    disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC_DP/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC_DP/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');
    ErrorVector_F2(i) = 100*err_LC_DP/Peak_Ref_LC;
    dtVector2(i) = dt;
    i = i + 1;
end

 figure(2)
  plot4 = plot(dtVector2,ErrorVector_F2);
 legend('Load Change Error _ VBR','Load Change Error _ DQ')
 ylabel('Error [%]');
 xlabel('Time Step [s]');
 
 set(plot2,'LineWidth',2);
 set(plot4,'LineWidth',2);