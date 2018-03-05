clc
clear
%% read PLECS results

Results_Reference = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/Voltages_and_currents.csv');
%Te_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/electrical_torque.csv'); 
omega_Reference = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/omega.csv'); 
vt_Reference = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/vt.csv'); 
%theta_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/theta.csv'); 

%% Read data from DP simulation and calculate absolute value and phase

% Read values from CSV files
voltageDP = csvread('../../../vsa/Results/TestExciterAndTurbine/DPsim/DP/VBR/DP_SynchronGenerator_VBR_0.010000_LeftVector.csv',1);
%currentDP = csvread('../../../vsa/Results/ABCFault/DPsim/DP/VBR/DP_SynchronGenerator_VBR_0.003300_RightVector.csv',1);
Log_SynGen = csvread('../../../vsa/Results/TestExciterAndTurbine/DPsim/DP/VBR/SynGen_VBR_0.010000.csv',1);
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
%DPplotc = plot(currentShiftDP(:,1),-currentShiftDP(:,2));
DPabsPlotc = plot(currentAbsDP(:,1),currentAbsDP(:,2));
title('Current phase A');
%legend('Current Phase a Simulink', 'DP shift a', 'DP abs a')
legend('Current Phase a Simulink', 'DP abs a')
xlabel('time [s]')
ylabel('current [A]')

% Phase B
figure(5)
hold off
PLECSplot2c = plot(Results_Reference(:,1), Results_Reference(:,6), '--');
hold on
DPplot2c = plot(currentShiftDP(:,1),-currentShiftDP(:,3));
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
DPplot3c = plot(currentShiftDP(:,1),-currentShiftDP(:,4));
DPabsPlot3c = plot(currentAbsDP(:,1),currentAbsDP(:,4));
title('Currents phase C');
legend('Current Phase c Simulink', 'DP shift c', 'DP abs c')
xlabel('time [s]')
ylabel('current [A]')

figure(7)
hold off
plotomega1 = plot(Log_SynGen(:,1),Log_SynGen(:,9));
hold on
plotomega2 = plot(Results_Reference(:,1),omega_Reference*2*pi*60);
%title('Rotor speed');
legend('\omega DPSim','\omega Reference');
xlabel('Time [s]');
ylabel('\omega [rad/s]');

set(plotomega1,'LineWidth',2);
set(plotomega2,'LineWidth',2);
% 
figure(8)
hold off
plotvt1 = plot(Log_SynGen(:,1),Log_SynGen(:,11));
hold on
plotvt2 = plot(Results_Reference(:,1),vt_Reference);
%title('vt');
legend('Terminal Voltage DPSim','Terminal Voltage Reference');
xlabel('Time [s]');
ylabel('Terminal Voltage [V]');
set(plotvt1,'LineWidth',2);
set(plotvt2,'LineWidth',2);

l=length(currentShiftDP);
l_Ref = length(Results_Reference);
s = round(l_Ref/l);
l_new=round(1/3*l_Ref);

ReferenceCurrent_SS = Results_Reference(1:l_new,5);
ReferenceCurrent_F = Results_Reference(l_new+1:2*l_new-1,5);

    

    
if l == l_Ref
    CurrentVector_SS = -currentShiftDP(1:l_new,2);
    CurrentVector_LC = -currentShiftDP(l_new+1:2*l_new-1,2);
else
    CurrentVector_interpolated = interp(-currentShiftDP(:,2),s);
    CurrentVector_SS = CurrentVector_interpolated(1:l_new,:);
    CurrentVector_LC = CurrentVector_interpolated(l_new+1:2*l_new-1);
end
    
Dif_SS = abs(CurrentVector_SS - ReferenceCurrent_SS);
[MaxDif_SS,i1] = max(Dif_SS);
err_SS = sqrt(immse(CurrentVector_SS,ReferenceCurrent_SS));

Dif_F = abs(CurrentVector_LC - ReferenceCurrent_F);
[MaxDif_F,i1] = max(Dif_F);
err_F = sqrt(immse(CurrentVector_LC,ReferenceCurrent_F));
RMS_ref_F = rms(ReferenceCurrent_F);
    
    Peak_Ref_SS = 10209;
    %Peak_Ref_fault = 14650;
    %Peak_Ref_fault = max(ReferenceCurrent_F);
     Peak_Ref_fault = rms(ReferenceCurrent_F)*sqrt(2);
    
        disp(['   '])
    disp(['##################### Results for VBR Model ################################'])
    disp(['   '])
    disp(['STEADY STATE:'])
    disp(['  Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
    disp(['  Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);
    disp(['FAULT:'])
    disp(['  Maximum Error ia during fault: ', num2str(100*MaxDif_F/Peak_Ref_fault), ' %']);
    disp(['  Root Mean-squared error ia during fault: ', num2str(100*err_F/Peak_Ref_fault), ' %']);
