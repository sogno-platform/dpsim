filenameDP = 'LeftVectorLog_VarFreqRXLineResLoad';
filenameEMT = 'LeftVectorLog_VarFreqRXLineResLoadEMT';
path = '../VisualStudio/DPsimVS2015/Logs/';
finalTime = '_0.6';
timeStepRef = '_5e-05';
dataType = '.csv';

timeStep = '_0.001';
filenameVoltageDP = strcat(path, filenameDP, timeStep, finalTime, dataType);
filenameVoltageEMT = strcat(path, filenameEMT, timeStep, finalTime, dataType);
filenameRef = strcat(path, filenameEMT, timeStepRef, finalTime, dataType);

% Read from CSV files
voltageRef = csvread(filenameRef);
voltageDP = csvread(filenameVoltageDP);
compOffsetDP = (size(voltageDP,2) - 1) / 2;
voltageEMT = csvread(filenameVoltageEMT);

% Calculate DP absolute value
voltageAbsDP = voltageDP(:,1);
for col = 2:( compOffsetDP + 1 )
    for row = 1:size(voltageDP,1)
        voltageAbsDP(row,col) = sqrt(voltageDP(row,col)^2 + ...
            voltageDP(row,col+compOffsetDP)^2);
    end
end

% Shift DP values
voltageShiftDP = voltageDP(:,1);
for col = 2:(compOffsetDP + 1)
    for row = 1:size(voltageDP,1)
        voltageShiftDP(row,col) = voltageDP(row,col)*cos(2*pi*50*voltageDP(row,1)) - ...
            voltageDP(row,col+compOffsetDP)*sin(2*pi*50*voltageDP(row,1));
    end
end

% Downsampling of reference voltage
for row = 1:size(voltageShiftDP,1)
    if voltageRef(size(voltageRef,1),1) >= voltageShiftDP(row,1)
        indices(row) = find(voltageRef(:,1) == voltageShiftDP(row,1),1);        
    end
end
for i = 1:size(indices,2)
    row = indices(1,i)
    for col = 1:size(voltageRef,2)
        voltageRefAligned(i,col) = voltageRef(row,col);
    end
end

% Calculate MSEs
mseDP = sum((voltageRefAligned - voltageShiftDP).^2) / size(voltageRefAligned,1)
mseEMT = sum((voltageRefAligned - voltageEMT).^2) / size(voltageRefAligned,1)
