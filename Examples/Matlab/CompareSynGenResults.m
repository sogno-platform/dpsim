% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results

Results_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_Simulink/Voltages_and_currents.csv'); 
%Te_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/electrical_torque.csv'); 
%omega_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_Simulink/omega.csv'); 
%theta_PLECS = csvread('../../vsa/Results/SynGenVBREmt_ABCFault_PLECS/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../../vsa/Results/SynGenVbrEmt_ABCFault_DPsim/NewModel/EMT_SynchronGenerator_VBR_LeftVector.csv',1);
%CurrentVector = csvread('../../../vsa/Results/SynGenVbrEmt_ABCFault_DPsim/NewModel/1damping/EMT_SynchronGenerator_VBR_RightVector.csv',1);
Log_SynGen = csvread('../../../vsa/Results/SynGenVbrEmt_ABCFault_DPsim/NewModel/SynGen_gen.csv',1);
CurrentVector = Log_SynGen(:,1:4);
 %% Plot
figure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
plot(Results_PLECS(:,1),Results_PLECS(:,2),'--');

title('Voltage Phase a');
legend('va DPSim','va Simulink');

figure(2)
hold off
plot(VoltageVector(:,1),VoltageVector(:,3));
hold on
plot(Results_PLECS(:,1),Results_PLECS(:,3),'--');

title('Voltage Phase b');
legend('vb DPSim','vb Simulink');

figure(3)
hold off
plot(VoltageVector(:,1),VoltageVector(:,4));
hold on
plot(Results_PLECS(:,1),Results_PLECS(:,4),'--');

title('Voltage Phase c');
legend('vc DPSim','vc Simulink');

figure(4)
hold off
plot(CurrentVector(:,1),CurrentVector(:,2));
hold on
plot(Results_PLECS(:,1),-Results_PLECS(:,5),'--');
title('Current phase a');
legend('ia DPSim','ia Simulink');

figure(5)
hold off
plot(CurrentVector(:,1),CurrentVector(:,3));
hold on
plot(Results_PLECS(:,1),-Results_PLECS(:,6),'--');
title('Current phase b');
legend('ib DPSim','ib Simulink');

figure(6)
hold off
plot(CurrentVector(:,1),CurrentVector(:,4));
hold on
plot(Results_PLECS(:,1),-Results_PLECS(:,7),'--');
title('Current phase c');
legend('ic DPSim','ic Simulink');

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

% figure(7)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,9));
% hold on
% plot(Results_PLECS(:,1),omega_PLECS*2*pi*60);
% title('Rotor speed');
% legend('\omega DPSim','\omega Simulink');
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



%% Calculate and display error
NewLength = 5000;
TimeVectorResampled = linspace(0,0.3,NewLength);
VoltageVector_resampled = resample(VoltageVector(:,3),NewLength,length(VoltageVector(:,3)));
Voltage_PLECS_resampled = resample(Results_PLECS(:,3),NewLength,length(Results_PLECS(:,3)));
% Voltage phase a
Dif = abs(VoltageVector_resampled - Voltage_PLECS_resampled);
MaxDif = max(Dif);
err = sqrt(immse(VoltageVector_resampled,Voltage_PLECS_resampled));
disp(['Maximum Error vb: ', num2str(MaxDif), ' V']);
disp(['Root Mean-squared error vb: ', num2str(err), ' V']);
% figure(7)
% hold off
% plot(TimeVectorResampled,VoltageVector_resampled);
% hold on
% plot(TimeVectorResampled,Voltage_PLECS_resampled);
% plot(TimeVectorResampled,Dif);
% title('Resampled Voltages phase a');
% legend('DPsim','Simulink','Difference');
% current phase a
Current_resampled = resample(CurrentVector(:,3),NewLength,length(CurrentVector(:,3)));
Currents_PLECS_resampled = resample(Results_PLECS(:,6),NewLength,length(Results_PLECS(:,6)));
Dif2 = abs(-Current_resampled - Currents_PLECS_resampled);
MaxDif2 = max(Dif2);
err2 = sqrt(immse(-Current_resampled,Currents_PLECS_resampled));
disp(['Maximum Error ib: ', num2str(MaxDif2), ' A']);
disp(['Root Mean-squared error ib: ', num2str(err2), ' A']);
% figure(8)
% hold off
% plot(TimeVectorResampled,Current_resampled);
% hold on
% plot(TimeVectorResampled,Currents_PLECS_resampled);
% plot(TimeVectorResampled,Dif2);
% title('Resampled Currents phase a');
% legend('DPsim','Simulink','Difference');
% % Omega
% MaxDif2 = max(abs(Log_SynGen(:,9)) - omega_PLECS(1:1000000,1)*2*pi*60);
% err2 = immse(Log_SynGen(:,9),omega_PLECS(1:1000000,1)*2*pi*60);
% disp(['Maximum Error omega: ', num2str(MaxDif2), ' rad/s']);
% disp(['Mean-squared error omega: ', num2str(err2), ' rad/s']);

