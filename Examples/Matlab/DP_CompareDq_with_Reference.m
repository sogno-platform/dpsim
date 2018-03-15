clc
clear
%% read PLECS results

Results_Reference = csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
%Te_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/electrical_torque.csv'); 
%omega_PLECS = csvread('../../../vsa/Results/SynGenVbrEmt_ABCFault_PLECS/omega.csv'); 
%theta_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/theta.csv'); 
l_Ref = length(Results_Reference);

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Results_Reference(1:l_Ref/3,5));
Peak_Ref_LC = max(Results_Reference(l_Ref/3:2*l_Ref/3,5));

%% Read data from DP simulation and calculate absolute value and phase

% Read values from CSV files
voltageDP = csvread('../../../vsa/Results/LoadChange/DPsim/DP/Dq/DP_SynchronGenerator_Dq_0.000050_LeftVector.csv',1);
%currentDP = csvread('../../../vsa/Results/LoadChange/DPsim/DP/DP_SynchronGenerator_Dq_RightVector.csv',1);
Log_SynGen = csvread('../../../vsa/Results/LoadChange/DPsim/DP/Dq/SynGen_Dq_0.000050.csv',1);
currentDP = Log_SynGen(:,1:7);
compOffsetDP = (size(currentDP,2) - 1) / 2;

% Calculate Voltage DP absolute value
voltageAbsDP = voltageDP(:,1);
for col = 2:( compOffsetDP + 1 )
    for row = 1:size(voltageDP,1)
        voltageAbsDP(row,col) = sqrt(voltageDP(row,col)^2 + ...
            voltageDP(row,col+compOffsetDP)^2);
    end
end

% Voltage Shift DP values
voltageShiftDP = voltageDP(:,1);
for col = 2:(compOffsetDP + 1)
    for row = 1:size(voltageDP,1)
        voltageShiftDP(row,col) = voltageDP(row,col)*cos(2*pi*60*voltageDP(row,1)) - ...
            voltageDP(row,col+compOffsetDP)*sin(2*pi*60*voltageDP(row,1));
    end
end

% Calculate Current DP absolute value
currentAbsDP = currentDP(:,1);
for col = 2:( compOffsetDP + 1 )
    for row = 1:size(currentDP,1)
        currentAbsDP(row,col) = sqrt(currentDP(row,col)^2 + ...
            currentDP(row,col+compOffsetDP)^2);
    end
end

% Current Shift DP values
currentShiftDP = currentDP(:,1);
for col = 2:(compOffsetDP + 1)
    for row = 1:size(currentDP,1)
        currentShiftDP(row,col) = currentDP(row,col)*cos(2*pi*60*currentDP(row,1)) - ...
            currentDP(row,col+compOffsetDP)*sin(2*pi*60*currentDP(row,1));
    end
end


%% Plot Voltage

% Phase A
figure(1)
hold off
PLECSplot = plot(Results_Reference(:,1), Results_Reference(:,2), '--');
hold on
DPplot = plot(voltageShiftDP(:,1),voltageShiftDP(:,2));
DPabsPlot = plot(voltageAbsDP(:,1),voltageAbsDP(:,2));
title('Voltage A');
legend('Voltage Phase a Simulink', 'DP shift a', 'DP abs a')
xlabel('time [s]')
ylabel('voltage [V]')

% Phase B
figure(2)
hold off
PLECSplot2 = plot(Results_Reference(:,1), Results_Reference(:,3), '--');
hold on
DPplot2 = plot(voltageShiftDP(:,1),voltageShiftDP(:,3));
DPabsPlot2 = plot(voltageAbsDP(:,1),voltageAbsDP(:,3));
title('Voltage B');
legend('Voltage Phase b Simulink', 'DP shift b', 'DP abs b')
xlabel('time [s]')
ylabel('voltage [V]')

% Phase C
figure(3)
hold off
PLECSplot3 = plot(Results_Reference(:,1), Results_Reference(:,4), '--');
hold on
DPplot3 = plot(voltageShiftDP(:,1),voltageShiftDP(:,4));
DPabsPlot3 = plot(voltageAbsDP(:,1),voltageAbsDP(:,4));
title('Voltage C');
legend('Voltage Phase c Simulink', 'DP shift c', 'DP abs c')
xlabel('time [s]')
ylabel('voltage [V]')

%% Plot Current

% Phase A
figure(4)
hold off
PLECSplotc = plot(Results_Reference(:,1), Results_Reference(:,5), '--');
hold on
DPplotc = plot(currentShiftDP(:,1),currentShiftDP(:,2));
DPabsPlotc = plot(currentAbsDP(:,1),currentAbsDP(:,2));
title('Current phase A');
legend('Current Phase a Simulink', 'DP shift a', 'DP abs a')
xlabel('time [s]')
ylabel('current [A]')

% Phase B
figure(5)
hold off
PLECSplot2c = plot(Results_Reference(:,1), Results_Reference(:,6), '--');
hold on
DPplot2c = plot(currentShiftDP(:,1),currentShiftDP(:,3));
DPabsPlot2c = plot(currentAbsDP(:,1),currentAbsDP(:,3));
title('Current phase B');
legend('Current Phase b Simulink', 'DP shift b', 'DP abs b')
xlabel('time [s]')
ylabel('current [A]')

% Phase C
figure(6)
hold off
PLECSplot3c = plot(Results_Reference(:,1),Results_Reference(:,7), '--');
hold on
DPplot3c = plot(currentShiftDP(:,1),currentShiftDP(:,4));
DPabsPlot3c = plot(currentAbsDP(:,1),currentAbsDP(:,4));
title('Currents phase C');
legend('Current Phase c Simulink', 'DP shift c', 'DP abs c')
xlabel('time [s]')
ylabel('current [A]')

    
    l_DP=length(currentShiftDP);
    l_new_DP=round(1/3*l_DP);

    CurrentVector_SS_DP = currentShiftDP(1:l_new_DP,2);
    CurrentVector_LC_DP = currentShiftDP(l_new_DP:2*l_new_DP,2);

    CurrentReference_reduced = zeros(l_DP,2);
    
    if l_DP == l_Ref
        CurrentReference_reduced(:,1) = Results_Reference(:,1);
        CurrentReference_reduced(:,2) = Results_Reference(:,5);
    else
        s = round(l_Ref/l_DP);
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
    %% Calculate avarage step time
StepTimeVector = Log_SynGen(:,10);
disp(['Avarage step time for generator: ', num2str(mean(StepTimeVector)*1000), ' ms']);