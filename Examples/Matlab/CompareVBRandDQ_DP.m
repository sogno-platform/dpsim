clc
clear
%% read PLECS results

Results_Reference = csvread('../../../vsa/Results/ABCFault/Simulink/Voltages_and_currents.csv');

%% Read data from DP simulation and calculate absolute value and phase - VBR

% Read values from CSV files
Log_SynGen_VBR = csvread('../../../vsa/Results/ABCFault/DPsim/DP/VBR/SynGen_VBR_0.000050.csv',1);
currentDP_VBR = Log_SynGen_VBR(:,1:7);
compOffsetDP_VBR = (size(currentDP_VBR,2) - 1) / 2;

% Calculate Current DP absolute value
currentAbsDP_VBR = currentDP_VBR(:,1);
for col = 2:( compOffsetDP_VBR + 1 )
    for row = 1:size(currentDP_VBR,1)
        currentAbsDP_VBR(row,col) = sqrt(currentDP_VBR(row,col)^2 + ...
            currentDP_VBR(row,col+compOffsetDP_VBR)^2);
    end
end

% Current Shift DP values
currentShiftDP_VBR = currentDP_VBR(:,1);
for col = 2:(compOffsetDP_VBR + 1)
    for row = 1:size(currentDP_VBR,1)
        currentShiftDP_VBR(row,col) = currentDP_VBR(row,col)*cos(2*pi*60*currentDP_VBR(row,1)) - ...
            currentDP_VBR(row,col+compOffsetDP_VBR)*sin(2*pi*60*currentDP_VBR(row,1));
    end
end


%% Read data from DP simulation and calculate absolute value and phase - Dq

% Read values from CSV files
Log_SynGen_Dq = csvread('../../../vsa/Results/ABCFault/DPsim/DP/Dq/SynGen_Dq_0.000050.csv',1);
currentDP_Dq = Log_SynGen_Dq(:,1:7);
compOffsetDP_Dq = (size(currentDP_Dq,2) - 1) / 2;

% Calculate Current DP absolute value
currentAbsDP_Dq = currentDP_Dq(:,1);
for col = 2:( compOffsetDP_Dq + 1 )
    for row = 1:size(currentDP_Dq,1)
        currentAbsDP_Dq(row,col) = sqrt(currentDP_Dq(row,col)^2 + ...
            currentDP_Dq(row,col+compOffsetDP_Dq)^2);
    end
end

% Current Shift DP values
currentShiftDP_Dq = currentDP_Dq(:,1);
for col = 2:(compOffsetDP_Dq + 1)
    for row = 1:size(currentDP_Dq,1)
        currentShiftDP_Dq(row,col) = currentDP_Dq(row,col)*cos(2*pi*60*currentDP_Dq(row,1)) - ...
            currentDP_Dq(row,col+compOffsetDP_Dq)*sin(2*pi*60*currentDP_Dq(row,1));
    end
end


%% Plot Current

% Phase A
figure(4)
hold off
PLECSplotc = plot(Results_Reference(:,1), Results_Reference(:,5), '--');
hold on
plot(currentShiftDP_VBR(:,1),-currentShiftDP_VBR(:,2));
plot(currentAbsDP_VBR(:,1),currentAbsDP_VBR(:,2));
plot(currentShiftDP_Dq(:,1),currentShiftDP_Dq(:,2));
 plot(currentAbsDP_Dq(:,1),currentAbsDP_Dq(:,2));
title('Current phase A');
legend('Current Phase a Reference', 'DP shift a VBR', 'DP abs a VBR', 'DP shift a Classical', 'DP abs a Classical')
xlabel('time [s]')
ylabel('current [A]')

