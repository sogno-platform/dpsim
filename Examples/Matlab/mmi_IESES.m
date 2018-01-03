clear all;
close all;

filenameDP = 'LeftVectorLog_VarFreqRXLineResLoad';
filenameEMT = 'LeftVectorLog_VarFreqRXLineResLoadEMT';
path = '../VisualStudio/DPsimVS2017/Logs/';
finalTime = '_0.4';
timeStepRef = '_5e-05';
dataType = '.csv';

%%
fprintf('Time step 50 mic sec \r')
timeStep = '_0.001';
filenameVoltageDP = strcat(path, filenameDP, timeStep, dataType);
timeStep = '_5e-05';
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, dataType);
mseDP = plotDpEmtVoltage(filenameVoltageDP, filenameVoltageEMT, 3);
