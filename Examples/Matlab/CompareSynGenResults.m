% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/SynGenDq_ABCFault/Simulink_PLECS/SynGenDqEmt_ABCFault_Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);
Results_Reference = Results_Reference(1:l_Ref,:);
%Te_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_PLECS/electrical_torque.csv'); 
%omega_PLECS = csvread('../../../vsa/Results/SynGenDqEmt_ABCFault_Simulink/omega.csv'); 
%theta_PLECS = csvread('../../../vsa/Results/SynGenDq_ABCFault/Simulink_PLECS/SynGenDqEmt_ABCFault_300M_Simulink/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../../vsa/Results/SimpSynGen/DPsim/NewTest/EMT_SynchronGenerator_ThreePhaseFault_LeftVector.csv',1);
CurrentVector = csvread('../../../vsa/Results/SimpSynGen/DPsim/NewTest/EMT_SynchronGenerator_ThreePhaseFault_RightVector.csv',1);
%Log_SynGen = csvread('../../../vsa/Results/SimpSynGen/DPsim/NewTest/SynGen_gen.csv',1);
%CurrentVector = Log_SynGen(:,1:4);
 %% Plot
figure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
plot(Results_Reference(:,1),Results_Reference(:,2),'--');

title('Voltage Phase a');
legend('va DPSim','va Simulink');

figure(2)
hold off
plot(VoltageVector(:,1), VoltageVector(:,3));
hold on
plot(Results_Reference(:,1),Results_Reference(:,3),'--');

title('Voltage Phase b');
legend('vb DPSim','vb Simulink');

figure(3)
hold off
plot(VoltageVector(:,1),VoltageVector(:,4));
hold on
plot(Results_Reference(:,1),Results_Reference(:,4),'--');

title('Voltage Phase c');
legend('vc DPSim','vc Simulink');

figure(4)
hold off
plot(CurrentVector(:,1),CurrentVector(:,2));
hold on
plot(Results_Reference(:,1),Results_Reference(:,5),'--');
title('Current phase a');
legend('ia DPSim','ia Simulink');

figure(5)
hold off
plot(CurrentVector(:,1),CurrentVector(:,3));
hold on
plot(Results_Reference(:,1),Results_Reference(:,6),'--');
title('Current phase b');
legend('ib DPSim','ib Simulink');

figure(6)
hold off
plot(CurrentVector(:,1),CurrentVector(:,4));
hold on
plot(Results_Reference(:,1),Results_Reference(:,7),'--');
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



% %% Calculate and display error
% %Cut Current and Voltage vector to get steady state results
% l=length(VoltageVector);
% l_new=round(1/3*l);
% VoltageVector_SteadyState = VoltageVector(1:l_new,:);
% CurrentVector_SteadyState = CurrentVector(1:l_new,:);
% VoltageVector_Fault = VoltageVector(l_new+1:2*l_new,:);
% CurrentVector_Fault = CurrentVector(l_new+1:2*l_new,:);
% 
% %Cut PLECS Results to get results before fault clearing
% l_Ref=length(Results_Reference);
% l_Ref_new = round(1/3*l_Ref);
% Reference_SteadyState = Results_Reference(1:l_Ref_new,:);
% Reference_Fault = Results_Reference(l_Ref_new+1:2*l_Ref_new,:);
% RMS_ref_SteadyState = rms(Reference_SteadyState(:,2));
% RMS_ref_Fault = rms(Reference_Fault(:,2));
% 
% % % Voltage phase a steady state
% Dif_SS = abs(VoltageVector_SteadyState(:,2) - Reference_SteadyState(:,2));
% [MaxDif_SS,i1] = max(Dif_SS);
% err_SS = sqrt(immse(VoltageVector_SteadyState(:,2),Reference_SteadyState(:,2)));
% disp(['RMS va steady state: ', num2str(RMS_ref_SteadyState), ' V']);
% disp(['Maximum Error va steady state: ', num2str(MaxDif_SS), ' V']);
% disp(['Root Mean-squared error va steady state: ', num2str(err_SS), ' V']);
% disp(['Maximum Error va steady state: ', num2str(100*MaxDif_SS/RMS_ref_SteadyState), ' %']);
% disp(['Root Mean-squared error va steady state: ', num2str(100*err_SS/RMS_ref_SteadyState), ' %']);
% 
% % % Voltage phase a fault
% Dif_Fault = abs(VoltageVector_Fault(:,2) - Reference_Fault(:,2));
% [MaxDif_Fault,i2] = max(Dif_Fault);
% err_Fault = sqrt(immse(VoltageVector_Fault(:,2),Reference_Fault(:,2)));
% disp(['RMS va Fault: ', num2str(RMS_ref_Fault), ' V']);
% disp(['Maximum Error va Fault: ', num2str(MaxDif_Fault), ' V']);
% disp(['Root Mean-squared error va Fault: ', num2str(err_Fault), ' V']);
% disp(['Maximum Error va Fault: ', num2str(100*MaxDif_Fault/RMS_ref_Fault), ' %']);
% disp(['Root Mean-squared error va Fault: ', num2str(100*err_Fault/RMS_ref_Fault), ' %']);
% 
% % Voltage phase a
% Dif = abs(VoltageVector(:,2) - Results_Reference(:,2));
% [MaxDif,i3] = max(Dif);
% err = sqrt(immse(VoltageVector(:,2),Results_Reference(:,2)));
% disp(['Maximum Error va: ', num2str(MaxDif), ' V']);
% disp(['Root Mean-squared error va Fault: ', num2str(err), ' V']);
% 
% figure(7)
% hold off
% plot(VoltageVector(:,1),Dif);
% hold on
% plot(VoltageVector(i1,1),MaxDif_SS,'r*')
% plot(VoltageVector(i2 + l_new,1),MaxDif_Fault,'r*')
% plot(VoltageVector(i3,1),MaxDif,'r*')
% text(VoltageVector(i1,1), MaxDif_SS, sprintf('Maximum error SS = %6.3f', MaxDif_SS))
% text(VoltageVector(i2 + l_new,1), MaxDif_Fault, sprintf('Maximum error Fault = %6.3f', MaxDif_Fault))
% text(VoltageVector(i3,1), MaxDif, sprintf('Maximum error = %6.3f', MaxDif))



% Va_PLECS_resampled = resample(PLECS_resampled(:,2),l_new,length(PLECS_resampled(:,2)));
