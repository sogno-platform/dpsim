% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/ABCFault/PLECS/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);
%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Results_Reference(1:l_Ref/3,5));
Peak_Ref_LC = max(Results_Reference(l_Ref/3:2*l_Ref/3,5));
%Results_Reference = Results_Reference(1:l_Ref,:);
%Te_Reference = csvread('../../../vsa/Results/ABCFault/Simulink/Te.csv'); 
%omega_Reference = csvread('../../../vsa/Results/ABCFault/Simulink/omega.csv'); 
%theta_PLECS = csvread('../../../vsa/Results/SynGenDq_ABCFault/Sim-0.81113286269894136ulink_PLECS/SynGenDqEmt_ABCFault_300M_Simulink/theta.csv'); 
%% read results from c++ simulation
VoltageVector = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/Dq/1DampingWinding/EMT_SynchronGenerator_Dq_0.000500_LeftVector.csv',1);
%CurrentVector = csvread('../../../vsa/Results/MultimachineTest/DPsim/EMT_SynchronGenerator_VBR_RightVector.csv',1);
Log_SynGen = csvread('../../../vsa/Results/ABCFault/DPsim/EMT/Dq/1DampingWinding/SynGen_Dq_0.000500.csv',1);
CurrentVector = Log_SynGen(:,1:4);
dt = 0.000500;
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
l=length(CurrentVector);
    l_new=round(1/3*l);

    CurrentVector_SS = CurrentVector(1:l_new,2);
    CurrentVector_LC = CurrentVector(l_new:2*l_new,2);

    CurrentReference_reduced = zeros(l,2);
    
    if l == l_Ref
     CurrentReference_reduced(:,1) = Results_Reference(:,1);
     CurrentReference_reduced(:,2) = Results_Reference(:,5);
    else
        s = round(dt/5e-5);
        n = 1;
        for m = 1:s:l_Ref
            CurrentReference_reduced(n,1) = Results_Reference(m,1);
            CurrentReference_reduced(n,2) = Results_Reference(m,5);
            n = n+1;
        end
    end  
 
    %Reference current in Steady state and after load change
    Reference_SS = CurrentReference_reduced(1:l_new,2);
    Reference_LC = CurrentReference_reduced(l_new:2*l_new,2);

    % Calculate maximum error and root mean squared error for steady state
    Dif_SS = abs(CurrentVector_SS - Reference_SS);
    [MaxDif_SS,i1] = max(Dif_SS);
    err_SS = sqrt(immse(CurrentVector_SS,Reference_SS));
    disp('############ Error for Dq EMT model ###############');
    disp(['############     dt= ',num2str(dt),'         ###############']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC = abs(CurrentVector_LC - Reference_LC);
    [MaxDif_LC,i1] = max(Dif_LC);
    err_LC = sqrt(immse(CurrentVector_LC,Reference_LC));
    disp(' ');
    disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');



%% Calculate avarage step time
StepTimeVector = Log_SynGen(:,7);
disp(['Avarage step time for generator: ', num2str(mean(StepTimeVector)*1000), ' ms']);