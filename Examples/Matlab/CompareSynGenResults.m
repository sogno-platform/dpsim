% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation

%% read PLECS results

Results_PLECS = csvread('../../../vsa/Results/TestExciterAndTurbine/SynGenDqEMTExciter_loadChange_Simulink/Voltages_and_currents.csv'); 
%Te_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/electrical_torque.csv'); 
omega_PLECS = csvread('../../../vsa/Results/TestExciterAndTurbine/SynGenDqEMTExciter_loadChange_Simulink/omega.csv'); 
%theta_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../../vsa/Results/TestExciterAndTurbine/SynGenVBREmtExciter_LoadChange_DPsim/data_vt.csv');
CurrentVector = csvread('../../../vsa/Results/TestExciterAndTurbine/SynGenVBREmtExciter_LoadChange_DPsim/data_j.csv');
Log_SynGen = csvread('../../../vsa/Results/TestExciterAndTurbine/SynGenVBREmtExciter_LoadChange_DPsim/SynGen_gen.csv');
 %% Plot
figure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
%plot(VoltageVector(:,1),VoltageVector(:,3));
%plot(VoltageVector(:,1),VoltageVector(:,4));

plot(Results_PLECS(:,1),Results_PLECS(:,2),'--');
%plot(Results_PLECS(:,1),Results_PLECS(:,3),'--');
%plot(Results_PLECS(:,1),Results_PLECS(:,4),'--');

title('Phase Voltages');
%legend('va DPSim','vb DPSim', 'vc DPSim','va PLECS','vb PLECS','vc PLECS');
legend('va DPSim','va Simulink');

figure(2)
hold off
plot(CurrentVector(:,1),CurrentVector(:,2));
hold on
%plot(CurrentVector(:,1),CurrentVector(:,3));
%plot(CurrentVector(:,1),CurrentVector(:,4));

plot(Results_PLECS(:,1),Results_PLECS(:,5),'--');
%plot(Results_PLECS(:,1),Results_PLECS(:,6),'--');
%plot(Results_PLECS(:,1),Results_PLECS(:,7),'--');

title('Phase Currents');
%legend('ia DPSim','ib DPSim','ic DPSim','ia PLECS','ib PLECS','ic PLECS');
legend('ia DPSim','ia Simulink');

% figure(3)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,2));
% hold on
% plot(Log_SynGen(:,1),Log_SynGen(:,3));
% plot(Log_SynGen(:,1),Log_SynGen(:,4));
% title ('Fluxes');
% legend('q','d','fd');
% 
% figure(4)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,5));
% hold on
% plot(Log_SynGen(:,1),Log_SynGen(:,6));
% plot(Log_SynGen(:,1),Log_SynGen(:,7));
% title ('dq voltages');
% legend('q','d','fd');
% 
% figure(5)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,8));
% hold on
% plot(Log_SynGen(:,1),Log_SynGen(:,9));
% plot(Log_SynGen(:,1),Log_SynGen(:,10));
% title ('dq currents');
% legend('q','d','fd');

figure(3)
hold off
plot(Log_SynGen(:,1),Log_SynGen(:,9));
hold on
plot(Results_PLECS(:,1),omega_PLECS);

title('Rotor speed');
legend('\omega DPSim','\omega Simulink');
% % 
% figure(4)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,20));
% hold on
% plot(Results_PLECS(:,1),-Te_PLECS);
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