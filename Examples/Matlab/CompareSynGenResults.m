% Compare Results of DPsim EMT with Simulink
% Plot Currents and Voltages and calculate errors for current

clc
clear all

%% Inputs
TestName = 'MultimachineTest'; % ABCFault, LoadChange, MultimachineTest
GeneratorType = 'Dq'; % VBR, Dq, DqSimplified
SimulationType = 'EMT'; % EMT
TimeStep = 0.000050;
StringTimeStep = '0.000050';
ShowVoltagePlots = 0;


%% read results
Results_Reference= csvread(['../../../vsa/Results/',TestName,'/Simulink/Voltages_and_currents.csv']);
l_Ref = length(Results_Reference);

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Results_Reference(1:l_Ref/3,5));
Peak_Ref_LC = max(Results_Reference(l_Ref/3:2*l_Ref/3,5));
if strcmp(TestName,'TestExciterAndTurbine') == 1
omega_Reference = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/omega.csv'); 
vt_Reference = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/vt.csv'); 
end
%% read results from c++ simulation
Path = ['../../../vsa/Results/',TestName,'/DPsim/',SimulationType,'/',GeneratorType,'/'];
FileName = ['SynGen_',SimulationType,'_',GeneratorType,'_',StringTimeStep,'.csv'];
FileName2 = [SimulationType,'_SynchronGenerator_',GeneratorType,'_',StringTimeStep,'_LeftVector.csv'];
Log_SynGen = csvread([Path,FileName],1);
VoltageVector = csvread([Path,FileName2],1);
CurrentVector = Log_SynGen(:,1:4);


%% Plot
if ShowVoltagePlots == 1
    figure(1)
    hold off
    plot(VoltageVector(:,1),VoltageVector(:,2));
    hold on
    plot(Results_Reference(:,1),Results_Reference(:,2),'--');
    title('Voltage Phase a');
    legend('va DPSim','va Reference');

    figure(2)
    hold off
    plot(VoltageVector(:,1), VoltageVector(:,3));
    hold on
    plot(Results_Reference(:,1),Results_Reference(:,3),'--');
    title('Voltage Phase b');
    legend('vb DPSim','vb Reference');

    figure(3)
    hold off
    plot(VoltageVector(:,1),VoltageVector(:,4));
    hold on
    plot(Results_Reference(:,1),Results_Reference(:,4),'--');
    title('Voltage Phase c');
    legend('vc DPSim','vc Reference');
end

h4 = figure(4)
hold off
if strcmp(GeneratorType,'VBR') == 1 || strcmp(GeneratorType,'VBRSimplified') == 1
    plot1=plot(CurrentVector(:,1),-CurrentVector(:,2));
else
    plot1=plot(CurrentVector(:,1),CurrentVector(:,2));
end
hold on
plot2 = plot(Results_Reference(:,1),Results_Reference(:,5),'--');
%title('Current phase a');
legend({'ia DP VBR','ia Reference'},'FontSize',12);
xlabel('Time [s]','FontSize',12);
ylabel('Current [A]','FontSize',12);

xlim([0 0.3]);

set(plot1,'LineWidth',2);
set(plot2,'LineWidth',2);

set(h4,'Units','centimeters');
set(h4,'pos',[5 5 24 13])
pos = get(h4,'Position');
set(h4,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

figure(5)
hold off
if strcmp(GeneratorType,'VBR') == 1
    plot(CurrentVector(:,1),-CurrentVector(:,3));
else
    plot(CurrentVector(:,1),CurrentVector(:,3));
end
hold on
plot(Results_Reference(:,1),Results_Reference(:,6),'--');
title('Current phase b');
legend('ib DPSim','ib Reference');

figure(6)
hold off
if strcmp(GeneratorType,'VBR') == 1
    plot(CurrentVector(:,1),-CurrentVector(:,4));
else
    plot(CurrentVector(:,1),CurrentVector(:,4));
end
hold on
plot(Results_Reference(:,1),Results_Reference(:,7),'--');
title('Current phase c');
legend('ic DPSim','ic Simulink');

if strcmp(TestName,'TestExciterAndTurbine') == 1 
h7=figure(7)
hold off
plotomega1 = plot(Log_SynGen(:,1),Log_SynGen(:,6));
hold on
plotomega2 = plot(Results_Reference(:,1),omega_Reference*2*pi*60);
%title('Rotor speed');
legend({'\omega DPSim','\omega Reference'},'FontSize',12);
xlabel('Time [s]','FontSize',12);
ylabel('\omega [rad/s]','FontSize',12);

set(plotomega1,'LineWidth',2);
set(plotomega2,'LineWidth',2);

set(h7,'Units','centimeters');
set(h7,'pos',[5 5 24 13])
pos = get(h7,'Position');
set(h7,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

h8 = figure(8)
hold off
plotvt1 = plot(Log_SynGen(:,1),Log_SynGen(:,8));
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





%% Calculate and display error
%Cut Current and Voltage vector to get steady state results

l=length(CurrentVector);
l_new=round(1/3*l);

if strcmp(GeneratorType,'VBR') == 1 || strcmp(GeneratorType,'VBRSimplified') == 1
    CurrentVector_SS = -CurrentVector(1:l_new,2);
    CurrentVector_LC = -CurrentVector(l_new:2*l_new,2);
else
    CurrentVector_SS = CurrentVector(1:l_new,2);
    CurrentVector_LC = CurrentVector(l_new:2*l_new,2);
end
    

CurrentReference_reduced = zeros(l,2);
    
if l == l_Ref
    CurrentReference_reduced(:,1) = Results_Reference(:,1);
    CurrentReference_reduced(:,2) = Results_Reference(:,5);
else
    s = round(TimeStep/5e-5);
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



% Calculate maximum error and root mean squared error for steady state
Dif_SS = abs(CurrentVector_SS - Reference_SS);
[MaxDif_SS,i1] = max(Dif_SS);
%err_SS = sqrt(immse(CurrentVector_SS,Reference_SS));
err_SS = sqrt( mean((CurrentVector_SS - Reference_SS).^2));
disp(['############ Error for ',GeneratorType,' ',SimulationType,' model ###############]']);
disp(['############ dt = ',StringTimeStep,' ###############]']);
disp([' ']);
disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

% Calculate maximum error and root mean squared error after load change
Dif_LC = abs(CurrentVector_LC - Reference_LC);
[MaxDif_LC,i1] = max(Dif_LC);
%err_LC = sqrt(immse(CurrentVector_LC,Reference_LC));
err_LC = sqrt( mean((CurrentVector_LC - Reference_LC).^2));
disp(' ');
disp(['Maximum Error ia ',TestName,': ', num2str(100*MaxDif_LC/Peak_Ref_LC), ' %']);
disp(['Root Mean-squared error ia ',TestName,': ', num2str(100*err_LC/Peak_Ref_LC), ' %']);
disp(' ');
disp(' ');

%% Calculate avarage step time
StepTimeVector = Log_SynGen(:,7);
disp(['Avarage step time for generator: ', num2str(mean(StepTimeVector)*1000), ' ms']);