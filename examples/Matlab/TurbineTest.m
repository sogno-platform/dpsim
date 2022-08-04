% Compare turbine outputs

%% read Simulink results

Results_Simulink = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/TurbineOutput.csv'); 
Omega_Simulink = csvread('../../../vsa/Results/TestExciterAndTurbine/Simulink/omega.csv'); 

%% read results from c++ simulation
Results_DPsim = csvread('../../../vsa/Results/TestExciterAndTurbine/DPsim/EMT/VBR/TurbineOutput_DPsim.csv');

%% Plot
figure(1)
hold off
plot(Results_DPsim(:,1),Results_DPsim(:,2).*Omega_Simulink);
hold on
plot(Results_DPsim(:,1),Results_Simulink(:,1),'--');

% Calculate and plot difference between results
AbsError = Results_DPsim(:,2) - Results_Simulink(:,1);
%plot(Results_DPsim(:,1),AbsError)
title('Turbine output');
legend('DPSim','Simulink','Difference');

%% Calculate and display error
RelError = AbsError ./ Results_Simulink(:,1);

MaxError = max(RelError);
err = immse(Results_DPsim(:,2),Results_Simulink(:,1));
disp(['Maximum Error: ', num2str(MaxError*100),  ' %']);
disp(['Mean-squared error: ', num2str(err)]);
