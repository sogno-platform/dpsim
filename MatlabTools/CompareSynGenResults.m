% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation

%% read PLECS results

%Results_PLECS = csvread('../../ReferenceExamples/PLECS/Symmetrical_fault.csv');  
Results_PLECS = csvread('../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/Voltages_and_currents.csv'); 
%Fluxes_PLECS = csvread('../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/Fluxes.csv'); 
%Te_PLECS = csvread('../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/electrical_torque.csv'); 

%% read results from c++ simulation
VoltageVector = csvread('../../vsa/Results/SynGenDqEmt_ABCFault_DPSim/data_vt.csv');
CurrentVector = csvread('../../vsa/Results/SynGenDqEmt_ABCFault_DPSim/data_j.csv');
 
 %% Plotfigure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
plot(VoltageVector(:,1),VoltageVector(:,3));
plot(VoltageVector(:,1),VoltageVector(:,4));

plot(Results_PLECS(:,1),Results_PLECS(:,2),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,3),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,4),'--');

title('Phase Voltages');
legend('va DPSim','vb DPSim', 'vc DPSim','va PLECS','vb PLECS','vc PLECS');

figure(2)
hold off
plot(CurrentVector(:,1),-CurrentVector(:,2));
hold on
plot(CurrentVector(:,1),-CurrentVector(:,3));
plot(CurrentVector(:,1),-CurrentVector(:,4));

plot(Results_PLECS(:,1),Results_PLECS(:,5),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,6),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,7),'--');

title('Phase Currents');
legend('ia DPSim','ib DPSim','ic DPSim','ia PLECS','ib PLECS','ic PLECS');

