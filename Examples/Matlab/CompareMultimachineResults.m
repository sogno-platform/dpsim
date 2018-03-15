% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read Reference results
Results_Reference= csvread('../../../vsa/Results/MultimachineTest/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);
omega_Reference = csvread('../../../vsa/Results/MultimachineTest/Simulink/omega.csv'); 
%% read results from c++ simulation
% EMT
Log_SynGen = csvread('../../../vsa/Results/MultimachineTest/DPsim/EMT/SynGen_gen.csv',1);
CurrentVector = Log_SynGen(:,1:4);

%% Read data from DP simulation and calculate absolute value and phase

% Read values from CSV files
Log_SynGenDP = csvread('../../../vsa/Results/MultimachineTest/DPsim/DP/SynGen_gen.csv',1);
currentDP = Log_SynGenDP(:,1:7);
compOffsetDP = (size(currentDP,2) - 1) / 2;

%% Plot results
figure(1)
hold off
plot1 = plot(CurrentVector(:,1),-CurrentVector(:,2));
hold on
plot2 = plot(Results_Reference(:,1),Results_Reference(:,5),'--');
legend('ia DPSim','ia Reference');
xlabel('Time [s]');
ylabel('Current [A]');

set(plot1,'LineWidth',2);
set(plot2,'LineWidth',2);


%% Calculate and display error
%Cut Current and Voltage vector to get steady state results
l=length(CurrentVector);
l_new=round(1/3*l);

CurrentVector_SS = -CurrentVector(1:l_new,2);
CurrentVector_LC = -CurrentVector(l_new:2*l_new,2);

CurrentReference_reduced = zeros(l,2);
    
if l == l_Ref
    CurrentReference_reduced(:,1) = Results_Reference(:,1);
    CurrentReference_reduced(:,2) = Results_Reference(:,5);
else
    s = round(l_Ref/l);
    n = 1;
    for m = 1:s:l_Ref
    CurrentReference_reduced(n,1) = Results_Reference(m,1);
    CurrentReference_reduced(n,2) = Results_Reference(m,5);
    n = n+1;
    end
end  
 
%Reference current in Steady state and after load change
Reference_SS = CurrentReference_reduced(1:l_new,2);
Reference_LC = CurrentReference_reduced(l_new:2*l_new,2);

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Reference_SS);
Peak_Ref_LC = max(Reference_LC);

% Calculate maximum error and root mean squared error for steady state
Dif_SS = abs(CurrentVector_SS - Reference_SS);
[MaxDif_SS,i1] = max(Dif_SS);
err_SS = sqrt(immse(CurrentVector_SS,Reference_SS));
disp('############ Error for VBR EMT model ###############');
disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

% Calculate maximum error and root mean squared error after load change
Dif_LC = abs(CurrentVector_LC - Reference_LC);
[MaxDif_LC,i1] = max(Dif_LC);
err_LC = sqrt(immse(CurrentVector_LC,Reference_LC));
disp(' ');
disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC/Peak_Ref_LC), ' %']);
disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC/Peak_Ref_LC), ' %']);

%% Calculate avarage step time
StepTimeVector = Log_SynGen(:,7);
disp(['Avarage step time for generator: ', num2str(mean(StepTimeVector)*1000), ' ms']);

figure(2)
hold off
plot(CurrentVector(l_new:2*l_new,1),CurrentVector_LC);
hold on
plot(CurrentVector(l_new:2*l_new,1),Reference_LC);
legend('ia DPSim','ia Reference','Max Error');



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


%% Plot Current

% Phase A
figure(4)
hold off
PLECSplotc = plot(Results_Reference(:,1), Results_Reference(:,5), '--');
hold on
DPplotc = plot(currentShiftDP(:,1),-currentShiftDP(:,2));
DPabsPlotc = plot(currentAbsDP(:,1),currentAbsDP(:,2));
legend('Current Phase a Simulink', 'DP shift a', 'DP abs a')
xlabel('time [s]')
ylabel('current [A]')
set(PLECSplotc,'LineWidth',2);
set(DPplotc,'LineWidth',2);
set(DPabsPlotc,'LineWidth',2);




%% Calculate and display error
l_DP=length(currentShiftDP);
l_new_DP=round(1/3*l_DP);

CurrentVector_SS_DP = -currentShiftDP(1:l_new_DP,2);
CurrentVector_LC_DP = -currentShiftDP(l_new_DP:2*l_new_DP,2);

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

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Reference_SS);
Peak_Ref_LC = max(Reference_LC);

% Calculate maximum error and root mean squared error for steady state
Dif_SS_DP = abs(CurrentVector_SS_DP - Reference_SS);
[MaxDif_SS_DP,i1] = max(Dif_SS_DP);
err_SS_DP = sqrt(immse(CurrentVector_SS_DP,Reference_SS));
disp('############ Error for VBR DP model ###############');
disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS_DP/Peak_Ref_SS), ' %']);
disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS_DP/Peak_Ref_SS), ' %']);

% Calculate maximum error and root mean squared error after load change
Dif_LC_DP = abs(CurrentVector_LC_DP - Reference_LC);
[MaxDif_LC_DP,i1] = max(Dif_LC_DP);
err_LC_DP = sqrt(immse(CurrentVector_LC_DP,Reference_LC));
disp(' ');
disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC_DP/Peak_Ref_LC), ' %']);
disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC_DP/Peak_Ref_LC), ' %']);