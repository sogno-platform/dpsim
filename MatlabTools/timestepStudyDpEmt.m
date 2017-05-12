clear all;
close all;

filenameDP = 'LeftVectorLog_VarFreqRXLineResLoad';
filenameEMT = 'LeftVectorLog_VarFreqRXLineResLoadEMT';
path = '../VisualStudio/DPsimVS2015/Logs/';
finalTime = '_0.6';
timeStep = '_5e-05';
timeStepRef = '_5e-05';
dataType = '.csv';

filenameRef = strcat(path, filenameEMT, timeStepRef, finalTime, dataType);

%%
fprintf('Time step 50 mic sec \r')
timeStep = '_5e-05';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
mseDP = plotDpEmtVoltage(filenameVoltageDP, filenameVoltageEMT, 3);
comparison = [0.05 0 0 mseDP(4) 0]
%%
fprintf('Time step 1 ms \r')
timeStep = '_0.001';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(2,:) = [1 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]

%%
fprintf('Time step 5 ms \r')
timeStep = '_0.005';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(3,:) = [5 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]

%%
fprintf('Time step 10 ms \r')
timeStep = '_0.01';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(4,:) = [10 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]

%%
fprintf('Time step 15 ms \r')
timeStep = '_0.015';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(5,:) = [15 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]

%%
fprintf('Time step 20 ms \r')
timeStep = '_0.02';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(6,:) = [20 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]

%%
fprintf('Time step 25 ms \r')
timeStep = '_0.025';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(7,:) = [25 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]

%%
fprintf('Time step 30 ms \r')
timeStep = '_0.03';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(8,:) = [30 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]
%%
fprintf('Time step 35 ms \r')
timeStep = '_0.035';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(9,:) = [35 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]
%%
fprintf('Time step 40 ms \r')
timeStep = '_0.04';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
[mseDP, mseEMT] = compareDpAndEmt(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
[mseDPInterp, mseEMTInterp] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, 3);
comparison(10,:) = [40 mseEMT(4) mseEMTInterp(4) mseDP(4) mseDPInterp(4)]

dlmwrite('comparison_table.csv',comparison,';')