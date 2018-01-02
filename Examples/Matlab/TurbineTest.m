% Compare turbine outputs

%% read Simulink results

Results_Simulink = csvread('../../../vsa/Results/TestExciterAndTurbine/TurbineTest/TurbineOutput_Simulink.csv'); 
Omega_Simulink = csvread('../../../vsa/Results/TestExciterAndTurbine/TurbineTest/omega.csv'); 

%% read results from c++ simulation
Results_DPsim = csvread('../../../vsa/Results/TestExciterAndTurbine/TurbineTest/TurbineOutput_DPsim.csv');

%% Plot
figure(1)
hold off
plot(Results_DPsim(:,1),Results_DPsim(:,2));
hold on
plot(Results_DPsim(:,1),Results_Simulink(:,1),'--');
title('Turbine output');
legend('DPSim','Simulink','Input(omega)');

AbsError = Results_DPsim(:,2) - Results_Simulink(:,1);
RelError = AbsError ./ Results_Simulink(:,1);
figure (2)
hold off
plot(Results_DPsim(:,1),RelError)