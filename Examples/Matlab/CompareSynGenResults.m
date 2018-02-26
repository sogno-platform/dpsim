% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/ABCFault/Simulink/Voltages_and_currents.csv');
%l_Ref = length(Results_Reference);
%Results_Reference = Results_Reference(1:l_Ref,:);
%Te_Reference = csvread('../../../vsa/Results/ABCFault/Simulink/Te.csv'); 
%omega_Reference = csvread('../../../vsa/Results/ABCFault/Simulink/omega.csv'); 
%theta_PLECS = csvread('../../../vsa/Results/SynGenDq_ABCFault/Sim-0.81113286269894136ulink_PLECS/SynGenDqEmt_ABCFault_300M_Simulink/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../../vsa/Results/ABCFault/DPsim/VBR/EMT_SynchronGenerator_VBR_0.000500_LeftVector.csv',1);
%CurrentVector = csvread('../../../vsa/Results/MultimachineTest/DPsim/EMT_SynchronGenerator_VBR_RightVector.csv',1);
Log_SynGen = csvread('../../../vsa/Results/ABCFault/DPsim/VBR/SynGen_VBR_0.0005.csv',1);
CurrentVector = Log_SynGen(:,1:4);
 %% Plot
figure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
plot(Results_Reference(:,1),Results_Reference(:,2),'--');

title('Voltage Phase a');
legend('va DPSim','va Reference');

figure(2)
hold off
plot(VoltageVector(:,1), VoltageVector(:,3));
hold on
plot(Results_Reference(:,1),Results_Reference(:,3),'--');

title('Voltage Phase b');
legend('vb DPSim','vb Reference');

figure(3)
hold off
plot(VoltageVector(:,1),VoltageVector(:,4));
hold on
plot(Results_Reference(:,1),Results_Reference(:,4),'--');

title('Voltage Phase c');
legend('vc DPSim','vc Reference');

figure(4)
hold off
plot(CurrentVector(:,1),-CurrentVector(:,2));
hold on
plot(Results_Reference(:,1),Results_Reference(:,5),'--');
title('Current phase a');
legend('ia DPSim','ia Reference');

figure(5)
hold off
plot(CurrentVector(:,1),-CurrentVector(:,3));
hold on
plot(Results_Reference(:,1),Results_Reference(:,6),'--');
title('Current phase b');
legend('ib DPSim','ib Reference');

figure(6)
hold off
plot(CurrentVector(:,1),-CurrentVector(:,4));
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
% plot(Log_SynGen(:,1),Log_SynGen(:,9));6
% plot(Log_SynGen(:,1),Log_SynGen(:,10));
% title ('dq currents');
% legend('q','d','fd');
% 
% figure(7)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,8));
% hold on
% plot(Results_Reference(:,1),omega_Reference*2*pi*60);
% title('Rotor speed');
% legend('\omega DPSim','\omega Reference');
% 
% figure(8)
% hold off
% plot(Log_SynGen(:,1),Log_SynGen(:,7));
% hold on
% plot(Results_Reference(:,1),-Te_Reference); 
% title('Electrical Torque');
%  legend('Te DPSim','Te Reference');




%% Calculate and display error
%Cut Current and Voltage vector to get steady state results
l=length(VoltageVector);
l_new=round(1/3*l);
VoltageVector_SteadyState = VoltageVector(1:l_new,:);
CurrentVector_SteadyState = -CurrentVector(1:l_new,:);
VoltageVector_Fault = VoltageVector(l_new+1:2*l_new,:);
CurrentVector_Fault = -CurrentVector(l_new+1:2*l_new,:);

%Cut Reference Results to get results before fault clearing
l_Ref=length(Results_Reference);
l_Ref_new = round(1/3*l_Ref);
Reference_SteadyState = Results_Reference(1:l_Ref_new,:);
Reference_Fault = Results_Reference(l_Ref_new+1:2*l_Ref_new,:);
RMS_ref_SteadyState = rms(Reference_SteadyState(:,5));
RMS_ref_Fault = rms(Reference_Fault(:,5));

% % Current phase a steady state
Dif_SS = abs(CurrentVector_SteadyState(:,2) - Reference_SteadyState(:,5));
[MaxDif_SS,i1] = max(Dif_SS);
err_SS = sqrt(immse(CurrentVector_SteadyState(:,2),Reference_SteadyState(:,5)));
disp(['RMS ia steady state: ', num2str(RMS_ref_SteadyState), ' A']);
disp(['Maximum Error ia steady state: ', num2str(MaxDif_SS), ' A']);
disp(['Root Mean-squared error ia steady state: ', num2str(err_SS), ' A']);
disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);
disp(['Root Mean-squared error va steady state: ', num2str(100*err_SS/(RMS_ref_SteadyState*sqrt(2))), ' %']);

% % Current phase a fault
Dif_Fault = abs(CurrentVector_Fault(:,2) - Reference_Fault(:,5));
[MaxDif_Fault,i2] = max(Dif_Fault);
err_Fault = sqrt(immse(CurrentVector_Fault(:,2),Reference_Fault(:,5)));
disp(['RMS ia Fault: ', num2str(RMS_ref_Fault), ' A']);
disp(['Maximum Error ia Fault: ', num2str(MaxDif_Fault), ' A']);
disp(['Root Mean-squared error ia Fault: ', num2str(err_Fault), ' A']);
disp(['Maximum Error va Fault: ', num2str(100*MaxDif_Fault/(RMS_ref_Fault*sqrt(2))), ' %']);
disp(['Root Mean-squared error ia Fault: ', num2str(100*err_Fault/(RMS_ref_Fault*sqrt(2))), ' %']);
% Va_PLECS_resampled = resample(PLECS_resampled(:,2),l_new,length(PLECS_resampled(:,2)));

%% Calculate avarage step time
StepTimeVector = Log_SynGen(:,7);
disp(['Avarage step time for generator: ', num2str(mean(StepTimeVector)*1000), ' ms']);