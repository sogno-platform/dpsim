% Compare exciter outputs

%% read Simulink results

Results_Simulink = csvread('../../../vsa/Results/MultimachineTest/Simulink/ExciterOutput_Simulink.csv'); 

%% read results from c++ simulation
Results_DPsim = csvread('../../../vsa/Results/MultimachineTest/DPsim/Exciter/ExciterOutput_DPSim.csv');

%% Plot
figure(1)
hold off
plot(Results_DPsim(:,1),Results_DPsim(:,2));
hold on
plot(Results_DPsim(:,1),Results_Simulink(:,1),'--');

% Calculate and plot difference between results
Difference = Results_DPsim(:,2) - Results_Simulink(:,1);
plot(Results_DPsim(:,1),Difference)
title('Exciter output');
legend('DPSim','Simulink','Difference');

%% Calculate and display error
RelError = Difference ./ Results_Simulink(:,1);
MaxDif = max(Difference);
MaxError = max(RelError);
err = immse(Results_DPsim(:,2),Results_Simulink(:,1));
disp(['Maximum Error (%): ', num2str(MaxError*100),  ' %']);
disp(['Maximum Error: ', num2str(MaxDif)]);
disp(['Mean-squared error: ', num2str(err)]);
