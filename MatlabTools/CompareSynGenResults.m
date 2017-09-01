% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation

%% read PLECS results

Results_PLECS = csvread('../../vsa/Results/SynGenVBREmt_Steady_PLECS/Voltages_and_currents.csv'); 
Te_PLECS = csvread('../../vsa/Results/SynGenVBREmt_Steady_PLECS/electrical_torque.csv'); 
omega_PLECS = csvread('../../vsa/Results/SynGenVBREmt_Steady_PLECS/omega.csv'); 
theta_PLECS = csvread('../../vsa/Results/SynGenVBREmt_Steady_PLECS/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../vsa/Results/SynGenVBREmt_Steady_DPSim/data_synGenVBR_v.csv');
CurrentVector = csvread('../../vsa/Results/SynGenVBREmt_Steady_DPSim/data_synGenVBR_i.csv');
omega = csvread('../../vsa/Results/SynGenVBREmt_Steady_DPSim/data_synGenVBR_omega.csv');
Te = csvread('../../vsa/Results/SynGenVBREmt_Steady_DPSim/data_synGenVBR_Te.csv');
theta = csvread('../../vsa/Results/SynGenVBREmt_Steady_DPSim/data_synGenVBR_theta.csv');
 %% Plot
figure(1)
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
plot(CurrentVector(:,1),CurrentVector(:,2));
hold on
plot(CurrentVector(:,1),CurrentVector(:,3));
plot(CurrentVector(:,1),CurrentVector(:,4));

plot(Results_PLECS(:,1),Results_PLECS(:,5),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,6),'--');
plot(Results_PLECS(:,1),Results_PLECS(:,7),'--');

title('Phase Currents');
legend('ia DPSim','ib DPSim','ic DPSim','ia PLECS','ib PLECS','ic PLECS');

figure(3)
hold off
plot(omega(:,1),omega(:,2)*2*pi*60);
hold on
plot(Results_PLECS(:,1),omega_PLECS);

title('Rotor speed');
legend('\omega DPSim','\omega PLECS');

figure(4)
hold off
plot(Te(:,1),Te(:,2)*18881.483433953665*51.979786748911749*3/2);
hold on
plot(Results_PLECS(:,1),Te_PLECS);

title('Electrical Torque');
legend('Te DPSim','Te PLECS');

figure(5)
hold off
plot(theta(:,1),theta(:,2));
hold on
plot(Results_PLECS(:,1),theta_PLECS);

title('Rotor position');
legend('\theta DPSim','\theta PLECS');