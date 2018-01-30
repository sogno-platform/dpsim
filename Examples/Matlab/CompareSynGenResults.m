% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results

Results_PLECS = csvread('../../../vsa/Results/SynGenDq_ABCFault/Simulink_PLECS/SynGenDqEmt_ABCFault_300M_Simulink/Voltages_and_currents.csv'); 
%Te_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/electrical_torque.csv'); 
%omega_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_Simulink/omega.csv'); 
theta_PLECS = csvread('../../../vsa/Results/SynGenDq_ABCFault/Simulink_PLECS/SynGenDqEmt_ABCFault_300M_Simulink/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../../vsa/Results/SynGenDq_ABCFault/DPsim/WithCompensation/300M/EMT_SynchronGenerator_ThreePhaseFault_LeftVector.csv',1);
%CurrentVector = csvread('../../../vsa/Results/SynGenDq_ABCFault/DPsim/WithCompensation/300M/EMT_SynchronGenerator_ThreePhaseFault_RightVector.csv',1);
Log_SynGen = csvread('../../../vsa/Results/SynGenDq_ABCFault/DPsim/WithCompensation/300M/SynGen_gen.csv',1);
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
plot(Results_PLECS(:,1),Results_PLECS(:,5),'--');
title('Current phase a');
legend('ia DPSim','ia Simulink');

figure(5)
hold off
plot(CurrentVector(:,1),CurrentVector(:,3));
hold on
plot(Results_PLECS(:,1),Results_PLECS(:,6),'--');
title('Current phase b');
legend('ib DPSim','ib Simulink');

figure(6)
hold off
plot(CurrentVector(:,1),CurrentVector(:,4));
hold on
plot(Results_PLECS(:,1),Results_PLECS(:,7),'--');
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
% figure(7)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,28));
% hold on
% plot(Results_PLECS(:,1),theta_PLECS.*pi/180);
% 
% title('Rotor position');
% legend('\theta DPSim','\theta PLECS');



%% Calculate and display error

l=length(VoltageVector);
l_new=round(2/3*l);

%Cut Current and Voltage vector to get results before fault clearing
VoltageVector_resampled = VoltageVector(1:l_new,:);
CurrentVector_resampled = CurrentVector(1:l_new,:);

%Cut PLECS Results to get results before fault clearing
l_PLECS=length(Results_PLECS);
l_PLECS_new = round(2/3*l_PLECS);
PLECS_resampled = Results_PLECS(1:l_PLECS_new,:);

Va_PLECS_resampled = resample(PLECS_resampled(:,2),l_new,length(PLECS_resampled(:,2)));
Vb_PLECS_resampled = resample(PLECS_resampled(:,3),l_new,length(PLECS_resampled(:,3)));
Vc_PLECS_resampled = resample(PLECS_resampled(:,4),l_new,length(PLECS_resampled(:,4)));

Ia_PLECS_resampled = resample(PLECS_resampled(:,5),l_new,length(PLECS_resampled(:,5)));
Ib_PLECS_resampled = resample(PLECS_resampled(:,6),l_new,length(PLECS_resampled(:,6)));
Ic_PLECS_resampled = resample(PLECS_resampled(:,7),l_new,length(PLECS_resampled(:,7)));

% % Voltage phase a
Dif = abs(VoltageVector_resampled(:,2) - Va_PLECS_resampled);
MaxDif = max(Dif);
err = sqrt(immse(VoltageVector_resampled(:,2),Va_PLECS_resampled));
disp(['Maximum Error va: ', num2str(MaxDif), ' V']);
disp(['Root Mean-squared error va: ', num2str(err), ' V']);

% % Voltage phase b
Dif2 = abs(VoltageVector_resampled(:,3) - Vb_PLECS_resampled);
MaxDif2 = max(Dif2);
err2 = sqrt(immse(VoltageVector_resampled(:,3),Vb_PLECS_resampled));
disp(['Maximum Error vb: ', num2str(MaxDif2), ' V']);
disp(['Root Mean-squared error vb: ', num2str(err2), ' V']);

% % Voltage phase c
Dif3 = abs(VoltageVector_resampled(:,4) - Vc_PLECS_resampled);
MaxDif3 = max(Dif3);
err3 = sqrt(immse(VoltageVector_resampled(:,4),Vc_PLECS_resampled));
disp(['Maximum Error vc: ', num2str(MaxDif3), ' V']);
disp(['Root Mean-squared error vc: ', num2str(err3), ' V']);

% % Current phase a
Dif4 = abs(CurrentVector_resampled(:,2) - Ia_PLECS_resampled);
MaxDif4 = max(Dif4);
err4 = sqrt(immse(CurrentVector_resampled(:,2),Ia_PLECS_resampled));
disp(['Maximum Error ia: ', num2str(MaxDif4), ' A']);
disp(['Root Mean-squared error ia: ', num2str(err4), ' A']);

% % Current phase b
Dif5 = abs(CurrentVector_resampled(:,3) - Ib_PLECS_resampled);
MaxDif5 = max(Dif5);
err5 = sqrt(immse(CurrentVector_resampled(:,3),Ib_PLECS_resampled));
disp(['Maximum Error ib: ', num2str(MaxDif5), ' A']);
disp(['Root Mean-squared error ib: ', num2str(err5), ' A']);

% % Current phase c
Dif6 = abs(CurrentVector_resampled(:,4) - Ic_PLECS_resampled);
MaxDif6 = max(Dif6);
err6 = sqrt(immse(CurrentVector_resampled(:,4),Ic_PLECS_resampled));
disp(['Maximum Error ic: ', num2str(MaxDif6), ' A']);
disp(['Root Mean-squared error ic: ', num2str(err6),' A']);