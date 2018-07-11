% Compare Results of DPsim DP with Simulink
% Plot Currents and Voltages and calculate errors for current

clc
clear

%% Inputs
TestName = 'MultimachineTest'; % ABCFault, LoadChange, MultimachineTest
GeneratorType = 'Dq'; % VBR or Dq
SimulationType = 'DP'; % DP
TimeStep = 0.000050;
StringTimeStep = '0.000050';
ShowVoltagePlots = 0;


%% read PLECS results

Results_Reference= csvread(['../../../vsa/Results/',TestName,'/Simulink/Voltages_and_currents.csv']);
if strcmp(TestName,'TestExciterAndTurbine') == 1
omega_Reference = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/omega.csv'); 
vt_Reference = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/vt.csv'); 
end
l_Ref = length(Results_Reference);

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Results_Reference(1:l_Ref/3,5));
Peak_Ref_LC = max(Results_Reference(l_Ref/3:2*l_Ref/3,5));

%% read results from c++ simulation

Path = ['../../../vsa/Results/',TestName,'/DPsim/',SimulationType,'/',GeneratorType,'/'];
FileName = ['SynGen_',SimulationType,'_',GeneratorType,'_',StringTimeStep,'.csv'];
FileName2 = [SimulationType,'_SynchronGenerator_',GeneratorType,'_',StringTimeStep,'_LeftVector.csv'];
Log_SynGen = csvread([Path,FileName],1);
voltageDP = csvread([Path,FileName2],1);
CurrentVector = Log_SynGen(:,1:4);

dt = TimeStep;
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

if ShowVoltagePlots == 1
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

end

%% Plot Current

% Phase A
h4 = figure(4)
hold off
PLECSplotc = plot(Results_Reference(:,1), Results_Reference(:,5));
hold on
if strcmp(GeneratorType,'VBR') == 1
    %DPplotc = plot(currentShiftDP(:,1),-currentShiftDP(:,2));
else
    %DPplotc = plot(currentShiftDP(:,1),currentShiftDP(:,2));
end
DPabsPlotc = plot(currentAbsDP(:,1),currentAbsDP(:,2));
%title('Current phase A');
%legend('Current Phase a Simulink', 'DP shift a', 'DP abs a')

set(PLECSplotc,'LineWidth',2);
set(DPabsPlotc,'LineWidth',2);
if strcmp(GeneratorType,'VBR') == 1
legend({'Reference', 'DP VBR abs'},'FontSize',12)
else
 legend({'Reference', 'DP Classical abs'},'FontSize',12) 
end
xlabel('Time [s]','FontSize',12);
ylabel('Current [A]','FontSize',12);
xlim([0 0.3])

set(h4,'Units','centimeters');
set(h4,'pos',[5 5 24 13])
pos = get(h4,'Position');
set(h4,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])


% Phase B
figure(5)
hold off
PLECSplot2c = plot(Results_Reference(:,1), Results_Reference(:,6), '--');
hold on
if strcmp(GeneratorType,'VBR') == 1
    DPplot2c = plot(currentShiftDP(:,1),-currentShiftDP(:,3));
else
    DPplot2c = plot(currentShiftDP(:,1),currentShiftDP(:,3));
end
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
if strcmp(GeneratorType,'VBR') == 1
    DPplot3c = plot(currentShiftDP(:,1),-currentShiftDP(:,4));
else
    DPplot3c = plot(currentShiftDP(:,1),currentShiftDP(:,4));
end
DPabsPlot3c = plot(currentAbsDP(:,1),currentAbsDP(:,4));
title('Currents phase C');
legend('Current Phase c Simulink', 'DP shift c', 'DP abs c')
xlabel('time [s]')
ylabel('current [A]')

if strcmp(TestName,'TestExciterAndTurbine') == 1
h7 = figure(7)
hold off
plotomega1 = plot(Log_SynGen(:,1),Log_SynGen(:,9));
hold on
plotomega2 = plot(Results_Reference(:,1),omega_Reference*2*pi*60);
legend({'\omega DPSim','\omega Reference'},'FontSize',12);
xlabel('Time [s]','FontSize',12);
ylabel('\omega [rad/s]','FontSize',12);

set(plotomega1,'LineWidth',2);
set(plotomega2,'LineWidth',2);

set(h7,'Units','centimeters');
set(h7,'pos',[5 5 24 13])
pos = get(h7,'Position');
set(h7,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
% 
h8 = figure(8)
hold off
plotvt1 = plot(Log_SynGen(:,1),Log_SynGen(:,11));
hold on
plotvt2 = plot(Results_Reference(:,1),vt_Reference);
%title('vt');
legend({'Terminal Voltage DPSim','Terminal Voltage Reference'},'FontSize',12);
xlabel('Time [s]','FontSize',12);
ylabel('Terminal Voltage [V]','FontSize',12);

set(plotvt1,'LineWidth',2);
set(plotvt2,'LineWidth',2);

set(h8,'Units','centimeters');
set(h8,'pos',[5 5 24 13])
pos = get(h8,'Position');
set(h8,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
end

l_DP=length(currentShiftDP);
l_new_DP=round(1/3*l_DP);

if strcmp(GeneratorType,'VBR') == 1
    CurrentVector_SS_DP = -currentShiftDP(1:l_new_DP,2);
    CurrentVector_LC_DP = -currentShiftDP(l_new_DP:2*l_new_DP,2);
else
    CurrentVector_SS_DP = currentShiftDP(1:l_new_DP,2);
    CurrentVector_LC_DP = currentShiftDP(l_new_DP:2*l_new_DP,2);  
end
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
    %err_SS_DP = sqrt(immse(CurrentVector_SS_DP,Reference_SS));
    err_SS_DP = sqrt( mean((CurrentVector_SS_DP - Reference_SS).^2))
    disp(['############ Error for ',GeneratorType,' ',SimulationType,' model ###############]']);
    disp(['############ dt = ',StringTimeStep,' ###############]']);
    disp([' ']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS_DP/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS_DP/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC_DP = abs(CurrentVector_LC_DP - Reference_LC);
    [MaxDif_LC_DP,i1] = max(Dif_LC_DP);
    %err_LC_DP = sqrt(immse(CurrentVector_LC_DP,Reference_LC));
    err_LC_DP = sqrt( mean((CurrentVector_LC_DP - Reference_LC).^2));
    disp(' ');
    disp(['Maximum Error ia ',TestName,': ', num2str(100*MaxDif_LC_DP/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia ',TestName,': ', num2str(100*err_LC_DP/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');
    
