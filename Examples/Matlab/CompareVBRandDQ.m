% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all

SimulationType = 'ABCFault';
%% read PLECS results
Results_Reference= csvread(['../../../vsa/Results/',SimulationType,'/Simulink/Voltages_and_currents.csv']);
%% read results from c++ simulation
Log_SynGenDq = csvread(['../../../vsa/Results/',SimulationType,'/DPsim/EMT/Dq/SynGen_EMT_Dq_0.000050.csv'],1);
CurrentVectorDq = Log_SynGenDq(:,1:4);
Log_SynGenVBR = csvread(['../../../vsa/Results/',SimulationType,'/DPsim/EMT/VBR/SynGen_EMT_VBR_0.000050.csv'],1);
CurrentVectorVBR = Log_SynGenVBR(:,1:4);
 %% Plot
h1=figure(1);
hold off
Refplot = plot(Results_Reference(:,1),Results_Reference(:,5));
hold on
Dqplot = plot(CurrentVectorDq(:,1),CurrentVectorDq(:,2),'--');
VBRplot = plot(CurrentVectorVBR(:,1),-CurrentVectorVBR(:,2),'--');

set(Dqplot,'LineWidth',2);
set(VBRplot,'LineWidth',2);
set(Refplot,'LineWidth',2);

set(h1,'Units','centimeters');
set(h1,'pos',[5 5 24 13])
pos = get(h1,'Position');
set(h1,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
print(h1,'filename','-dpdf','-r0')
xlim([0 0.3])

xlabel('time [s]','FontSize',12)
ylabel('current [A]','FontSize',12)
legend({'Reference','EMT Classical','EMT VBR'},'FontSize',12);

%% read results from c++ simulation
Log_SynGenDq2 = csvread(['../../../vsa/Results/',SimulationType,'/DPsim/EMT/Dq/SynGen_EMT_Dq_0.000500.csv'],1);
CurrentVectorDq2 = Log_SynGenDq2(:,1:4);
Log_SynGenVBR2 = csvread(['../../../vsa/Results/',SimulationType,'/DPsim/EMT/VBR/SynGen_EMT_VBR_0.000500.csv'],1);
CurrentVectorVBR2 = Log_SynGenVBR2(:,1:4);
 %% Plot
h2 = figure(2);
hold off
Refplot2 = plot(Results_Reference(:,1),Results_Reference(:,5));
hold on
Dqplot2 = plot(CurrentVectorDq2(:,1),CurrentVectorDq2(:,2),'--');
VBRplot2 = plot(CurrentVectorVBR2(:,1),-CurrentVectorVBR2(:,2),'--');

set(Dqplot2,'LineWidth',2);
set(VBRplot2,'LineWidth',2);
set(Refplot2,'LineWidth',2);

set(h2,'Units','centimeters');
set(h2,'pos',[5 5 24 13])
pos = get(h2,'Position');
set(h2,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

xlabel('time [s]','FontSize',12)
ylabel('current [A]','FontSize',12)
legend({'Reference','EMT Classical','EMT VBR'},'FontSize',12);