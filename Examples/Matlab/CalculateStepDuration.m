% This script read the step durantion of the synchronous generator and 
%calculate the average duration.

%% read 
TestName = 'LoadChange';
GeneratorType = 'VBR';
SimulationType = 'EMT';
TimeStep = '0.000500';
Path = ['../../../vsa/Results/',TestName,'/DPsim/',SimulationType,'/',GeneratorType,'/','RESERVE/'];
FileName = ['SynGen_',SimulationType,'_',GeneratorType,'_',TimeStep,'.csv'];
Log_SynGen = csvread([Path,FileName],1);


%% Calculate avarage step time
StepTimeVector = Log_SynGen(:,2);
disp(['Avarage step time for generator: ', num2str(mean(StepTimeVector)*1000), ' ms']);