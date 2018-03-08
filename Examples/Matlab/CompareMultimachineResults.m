% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/MultimachineTest/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);
%Results_Reference = Results_Reference(1:l_Ref,:);
omega_Reference = csvread('../../../vsa/Results/MultimachineTest/Simulink/omega.csv'); 
%Te_Reference = csvread('../../../vsa/Results/ABCFault/Simulink/Te.csv'); 
%theta_PLECS = csvread('../../../vsa/Results/SynGenDq_ABCFault/Sim-0.81113286269894136ulink_PLECS/SynGenDqEmt_ABCFault_300M_Simulink/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../../vsa/Results/MultimachineTest/DPsim/EMT/EMT_SynchronGenerator_VBR_LeftVector.csv',1);
%CurrentVector = csvread('../../../vsa/Results/MultimachineTest/DPsim/EMT_SynchronGenerator_VBR_RightVector.csv',1);
Log_SynGen = csvread('../../../vsa/Results/MultimachineTest/DPsim/EMT/SynGen_gen.csv',1);
CurrentVector = Log_SynGen(:,1:4);

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


figure(2)
hold off
plot(CurrentVector(:,1),-CurrentVector(:,3));
hold on
plot(Results_Reference(:,1),Results_Reference(:,6),'--');
title('Current phase b');
legend('ib DPSim','ib Reference');

figure(3)
hold off
plot(CurrentVector(:,1),-CurrentVector(:,4));
hold on
plot(Results_Reference(:,1),Results_Reference(:,7),'--');
title('Current phase c');
legend('ic DPSim','ic Simulink');


%% Calculate and display error
%Cut Current and Voltage vector to get steady state results
l=length(CurrentVector);
l_new=round(1/3*l_Ref);


if l == l_Ref
    CurrentVector_SteadyState = CurrentVector(1:l_new,2);
    CurrentVector_Fault = CurrentVector(l_new+1:3*l_new,2);
else
    s = round(l_Ref/l);
    CurrentVector_interpolated = interp(CurrentVector(:,2),s);
    CurrentVector_SteadyState = CurrentVector_interpolated(1:l_new,:);
    CurrentVector_Fault = CurrentVector_interpolated(l_new+1:3*l_new,:);  
end  
    
%Cut Reference Results to get results before fault clearing
Reference_SteadyState = Results_Reference(1:l_new,5);
Reference_Fault = Results_Reference(l_new+1:3*l_new,5);


Peak_Ref_SS = 10209;
Peak_Ref_fault = rms(Reference_Fault)*sqrt(2);

% % Current phase a steady state
Dif_SS = abs(-CurrentVector_SteadyState - Reference_SteadyState);
[MaxDif_SS,i1] = max(Dif_SS);
err_SS = sqrt(immse(-CurrentVector_SteadyState,Reference_SteadyState));
disp(['Maximum Error ia steady state: ', num2str(MaxDif_SS), ' A']);
disp(['Root Mean-squared error ia steady state: ', num2str(err_SS), ' A']);
disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
disp(['Root Mean-squared error va steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

% % Current phase a fault
Dif_Fault = abs(-CurrentVector_Fault - Reference_Fault);
[MaxDif_Fault,i2] = max(Dif_Fault);
err_Fault = sqrt(immse(-CurrentVector_Fault,Reference_Fault));
disp(['Maximum Error ia Fault: ', num2str(MaxDif_Fault), ' A']);
disp(['Root Mean-squared error ia Fault: ', num2str(err_Fault), ' A']);
disp(['Maximum Error ia Fault: ', num2str(100*MaxDif_Fault/Peak_Ref_fault), ' %']);
disp(['Root Mean-squared error ia Fault: ', num2str(100*err_Fault/Peak_Ref_fault), ' %']);

%% Calculate avarage step time
StepTimeVector = Log_SynGen(:,7);
disp(['Avarage step time for generator: ', num2str(mean(StepTimeVector)*1000), ' ms']);



%% Read data from DP simulation and calculate absolute value and phase

% Read values from CSV files
Log_SynGen = csvread('../../../vsa/Results/MultimachineTest/DPsim/DP/SynGen_gen.csv',1);
currentDP = Log_SynGen(:,1:7);
compOffsetDP = (size(currentDP,2) - 1) / 2;


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


l=length(currentShiftDP);
s = round(l_Ref/l);

ReferenceCurrent_SS = Results_Reference(1:l_new,5);
ReferenceCurrent_F = Results_Reference(l_new+1:3*l_new,5);

    

    
if l == l_Ref
    CurrentVector_SS = -currentShiftDP(1:l_new,2);
    CurrentVector_LC = -currentShiftDP(l_new+1:3*l_new,2);
else
    CurrentVector_interpolated = interp(-currentShiftDP(:,2),s);
    CurrentVector_SS = CurrentVector_interpolated(1:l_new,:);
    CurrentVector_LC = CurrentVector_interpolated(l_new+1:3*l_new);
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
