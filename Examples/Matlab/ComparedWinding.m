% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read results from c++ simulation
Log_SynGen = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/VBR/1DampingWinding/SynGen_VBR_0.000500.csv',1);
CurrentVector = Log_SynGen(:,1:4);

Log_SynGen2 = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/VBR/SynGen_VBR_0.000500.csv',1);
CurrentVector2 = Log_SynGen2(:,1:4);

figure(1)
hold off
plot1 = plot(CurrentVector2(:,1),-CurrentVector2(:,2));
hold on
plot2 = plot(CurrentVector(:,1),-CurrentVector(:,2));
%plot(Results_Reference(:,1),Results_Reference(:,5),'--');
title('Current phase a');
legend('ia DPsim with 2 q-windings','ia DPsim with 1 q-winding');
xlabel('Time [s]');
ylabel('Current [A]');

 set(plot1,'LineWidth',2);
 set(plot2,'LineWidth',2);

