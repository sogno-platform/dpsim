clc
clear
%% read PLECS results

Results_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/Voltages_and_currents.csv'); 
%Te_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/electrical_torque.csv'); 
%omega_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/omega.csv'); 
%theta_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/theta.csv'); 

%% Read data from DP simulation and calculate absolute value and phase

% Read values from CSV files
voltageDP = csvread('../../../vsa/Results/Testing/data_vt.csv');
currentDP = csvread('../../../vsa/Results/Testing/data_j.csv');
%Log_SynGen = csvread('../../vsa/Results//SynGenDqDynPh_ABCFault_DPsim/Euler/SynGen_gen.csv');
compOffsetDP = (size(voltageDP,2) - 1) / 2;

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
PLECSplot = plot(Results_PLECS(:,1), Results_PLECS(:,2), '--');
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
PLECSplot2 = plot(Results_PLECS(:,1), Results_PLECS(:,3), '--');
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
PLECSplot3 = plot(Results_PLECS(:,1), Results_PLECS(:,4), '--');
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
PLECSplotc = plot(Results_PLECS(:,1), Results_PLECS(:,5), '--');
hold on
DPplotc = plot(currentShiftDP(:,1),-currentShiftDP(:,2));
DPabsPlotc = plot(currentAbsDP(:,1),currentAbsDP(:,2));
title('Current phase A');
legend('Current Phase a Simulink', 'DP shift a', 'DP abs a')
xlabel('time [s]')
ylabel('current [A]')

% Phase B
figure(5)
hold off
PLECSplot2c = plot(Results_PLECS(:,1), Results_PLECS(:,6), '--');
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
PLECSplot3c = plot(Results_PLECS(:,1), Results_PLECS(:,7), '--');
hold on
DPplot3c = plot(currentShiftDP(:,1),-currentShiftDP(:,4));
DPabsPlot3c = plot(currentAbsDP(:,1),currentAbsDP(:,4));
title('Currents phase C');
legend('Current Phase c Simulink', 'DP shift c', 'DP abs c')
xlabel('time [s]')
ylabel('current [A]')

% figure(7)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,21));
% hold on
% plot(Results_PLECS(:,1),omega_PLECS);
% 
% title('Rotor speed');
% legend('\omega DPSim','\omega PLECS');
% 
% figure(8)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,20));
% hold on
% plot(Results_PLECS(:,1),Te_PLECS);
% 
% title('Electrical Torque');
% legend('Te DPSim','Te PLECS');
% 
% figure(9)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,22));
% hold on
% plot(Results_PLECS(:,1),theta_PLECS);
% 
% title('Rotor position');
% legend('\theta DPSim','\theta PLECS');

