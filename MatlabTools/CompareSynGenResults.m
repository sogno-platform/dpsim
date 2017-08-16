% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation

%% run simulink example and save variables
 a = sim('../../ReferenceExamples/PLECS/synchronousGeneratorPLECS.slx','SimulationMode','normal');
 va = a.get('VoltageA');
 vb = a.get('VoltageB');
 vc = a.get('VoltageC');
 ia = a.get('CurrentA');
 ib = a.get('CurrentB');
 ic = a.get('CurrentC');
 time = a.get('tout');
 
%% read results from c++ simulation
 VoltageVector = csvread('../VisualStudio/DPsimVS2017/data_vt.csv');
 CurrentVector=csvread('../VisualStudio/DPsimVS2017/data_j.csv');
 
 %% Plot
figure(1)
hold off
plot(VoltageVector(:,1),VoltageVector(:,2));
hold on
plot(VoltageVector(:,1),VoltageVector(:,3));
plot(VoltageVector(:,1),VoltageVector(:,4));
plot(time,va,'--');
plot(time,vb,'--');
plot(time,vc,'--');
title('Phase Voltages');
legend('va c++','vb c++', 'vc c++','va PLECS','vb PLECS','vc PLECS');

figure(2)
hold off
plot(CurrentVector(:,1),CurrentVector(:,2))
hold on
plot(CurrentVector(:,1),CurrentVector(:,3))
plot(CurrentVector(:,1),CurrentVector(:,4))
plot(time,-ia,'--');
plot(time,-ib,'--');
plot(time,-ic,'--');
title('Currents');
legend('ia c++','ib c++','ic c++','ia PLECS','ib PLECS','ic PLECS');

