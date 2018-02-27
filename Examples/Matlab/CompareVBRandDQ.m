% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/ABCFault/Simulink/Voltages_and_currents.csv');
%% read results from c++ simulation
Log_SynGenDq = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/Dq/SynGen_Dq_0.000050.csv',1);
CurrentVectorDq = Log_SynGenDq(:,1:4);
Log_SynGenVBR = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/VBR/SynGen_VBR_0.000050.csv',1);
CurrentVectorVBR = Log_SynGenVBR(:,1:4);
 %% Plot
figure(1)
hold off
plot(CurrentVectorDq(:,1),CurrentVectorDq(:,2));
hold on
plot(CurrentVectorVBR(:,1),-CurrentVectorVBR(:,2),'--');
plot(Results_Reference(:,1),Results_Reference(:,5),'--');


title('Current Phase a');
legend('ia DPsim Classical','ia DPsim VBR', 'ia Reference');

%% read results from c++ simulation
Log_SynGenDq2 = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/Dq/SynGen_Dq_0.000500.csv',1);
CurrentVectorDq2 = Log_SynGenDq2(:,1:4);
Log_SynGenVBR2 = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/VBR/SynGen_VBR_0.000500.csv',1);
CurrentVectorVBR2 = Log_SynGenVBR2(:,1:4);
 %% Plot
figure(2)
hold off
plot(CurrentVectorDq2(:,1),CurrentVectorDq2(:,2));
hold on
plot(CurrentVectorVBR2(:,1),-CurrentVectorVBR2(:,2),'--');
plot(Results_Reference(:,1),Results_Reference(:,5),'--');

title('Current Phase a');
legend('ia DPsim Classical','ia DPsim VBR', 'ia Reference');