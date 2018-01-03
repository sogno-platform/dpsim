% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation

%% read results from c++ simulation Full model
VoltageVector_full = csvread('../../vsa/Results/SynGenVbrEmt_ABCLongFault_DPsim/data_vt.csv');
CurrentVector_full = csvread('../../vsa/Results/SynGenVbrEmt_ABCLongFault_DPsim/data_j.csv');
Log_SynGen_full = csvread('../../vsa/Results/SynGenVbrEmt_ABCLongFault_DPsim/SynGen_gen.csv');
%% read results from c++ simulation
VoltageVector = csvread('../../vsa/Results/SimpSynGenVbrEmt_ABCLongFault_DPsim/data_vt.csv');
CurrentVector = csvread('../../vsa/Results/SimpSynGenVbrEmt_ABCLongFault_DPsim/data_j.csv');
Log_SynGen = csvread('../../vsa/Results/SimpSynGenVbrEmt_ABCLongFault_DPsim/SynGen_gen.csv');
 %% Plot
figure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
plot(VoltageVector_full(:,1),VoltageVector_full(:,2));
title('Voltage Phase a');
legend('va DPSim simplified model','va DPSim detailed model');

figure(2)
hold off
plot(VoltageVector(:,1),VoltageVector(:,3));
hold on
plot(VoltageVector_full(:,1),VoltageVector_full(:,3));
%plot(Results_Simulink(:,1),Results_Simulink(:,3),'--');
title('Voltage Phase b');
legend('vb DPSim simplified model','vb DPSim detailed model');

figure(3)
hold off
plot(VoltageVector(:,1),VoltageVector(:,4));
hold on
plot(VoltageVector_full(:,1),VoltageVector_full(:,4));
title('Voltage Phase c');
legend('vc DPSim simplified model','vc DPSim detailed model');


figure(4)
hold off
plot(CurrentVector(:,1),CurrentVector(:,2));
hold on
plot(CurrentVector_full(:,1),CurrentVector_full(:,2));
title('Current Phase a');
legend('ia DPSim simplified model','ia DPSim detailed model');

figure(5)
hold off
plot(CurrentVector(:,1),CurrentVector(:,3));
hold on
plot(CurrentVector_full(:,1),CurrentVector_full(:,3),'--');
title('Current Phase b');
legend('ib DPSim simplified model','ib DPSim detailed model');

figure(6)
hold off
plot(CurrentVector(:,1),CurrentVector(:,4));
hold on
plot(CurrentVector_full(:,1),CurrentVector_full(:,4),'--');
title('Current Phase c');
legend('ic DPSim simplified model','ic DPSim detailed model');

figure(7)
hold off
plot(Log_SynGen(:,1),Log_SynGen(:,9));
hold on
plot(Log_SynGen_full(:,1),Log_SynGen_full(:,9));
title('Rotor speed');
legend('\omega DPSim simplified model','\omega DPSim detailed model');
 
figure(8)
hold off
plot(Log_SynGen(:,1),Log_SynGen(:,8));
hold on
plot(Log_SynGen_full(:,1),Log_SynGen_full(:,8));
title('Electrical Torque');
legend('Te DPSim simplified model','Te DPSim detailed model');

