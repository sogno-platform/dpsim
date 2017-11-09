% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation

%% read PLECS results

Results_PLECS = csvread('../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/Voltages_and_currents.csv'); 
Te_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/electrical_torque.csv'); 
% omega_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/omega.csv'); 
%theta_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../vsa/Results/Testing/data_vt.csv');
CurrentVector = csvread('../../vsa/Results/Testing/data_j.csv');
%%Log_SynGen = csvread('../../vsa/Results/SynGenVbrEmt_ABCFault_DPsim_1_Damping/0.000001/SynGen_gen.csv');
 %% Plot
figure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
plot(VoltageVector(:,1),VoltageVector(:,3));
plot(VoltageVector(:,1),VoltageVector(:,4));

% plot(tout,voltages(:,1),'--')
% plot(tout,voltages(:,2),'--')
% plot(tout,voltages(:,3),'--')
plot(Results_PLECS(:,1),Results_PLECS(:,2),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,3),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,4),'--');

title('Phase Voltages');
legend('va DPSim','vb DPSim', 'vc DPSim','va PLECS','vb PLECS','vc PLECS');
%legend('va DPSim','vb DPSim', 'vc DPSim','va Simulink','vb Simulink','vc Simulink');

figure(2)
hold off
plot(VoltageVector(:,1),VoltageVector(:,5));
hold on
plot(VoltageVector(:,1),VoltageVector(:,6));
plot(VoltageVector(:,1),VoltageVector(:,7));

% plot(tout,currents(:,1),'--')
% plot(tout,currents(:,2),'--')
% plot(tout,currents(:,3),'--')


plot(Results_PLECS(:,1),Results_PLECS(:,5),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,6),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,7),'--');

title('Phase Currents');
legend('ia DPSim','ib DPSim','ic DPSim','ia PLECS','ib PLECS','ic PLECS');
%legend('ia DPSim','ib DPSim','ic DPSim','ia Simulink','ib Simulink','ic Simulink');

% figure(3)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,8));
% hold on
% plot(Results_PLECS(:,1),omega_PLECS);
% 
% title('Rotor speed');
% legend('\omega DPSim','\omega PLECS');
% 
% figure(4)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,7));
% hold on
% plot(Results_PLECS(:,1),Te_PLECS);
% 
% title('Electrical Torque');
% legend('Te DPSim','Te PLECS');
% 
% figure(5)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,4));
% hold on
% plot(Results_PLECS(:,1),theta_PLECS);
% 
% title('Rotor position');
% legend('\theta DPSim','\theta PLECS');