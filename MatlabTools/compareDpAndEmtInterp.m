function [ ] = compareDpAndEmtInterp(filenameRef, filenameVoltageDP, filenameVoltageEMT, plotNode)

% Increment node to skip time column
plotNode = plotNode + 1

% Read from CSV files
voltageRef = csvread(filenameRef);
voltageDP = csvread(filenameVoltageDP);
compOffsetDP = (size(voltageDP,2) - 1) / 2;
voltageEMT = csvread(filenameVoltageEMT);

% Interpolation
voltageDPInterp(:,1) =  voltageRef(:,1);

for col = 2:size(voltageDP,2)
    voltageDPInterp(:,col) =  interp1(voltageDP(:,1)',  voltageDP(:,col)', ...
        voltageDPInterp(:,1)', 'linear', 'extrap');
end

voltageEMTInterp(:,1) =  voltageRef(:,1);
for col = 2:size(voltageEMT,2)
    voltageEMTInterp(:,col) =  interp1(voltageEMT(:,1)', voltageEMT(:,col)', ...
        voltageEMTInterp(:,1)', 'linear', 'extrap')';
end

% Calculate DP absolute value
voltageAbsDP = voltageDP(:,1);
for col = 2:( compOffsetDP + 1 )
    for row = 1:size(voltageDP,1)
        voltageAbsDP(row,col) = sqrt(voltageDP(row,col)^2 + ...
            voltageDP(row,col+compOffsetDP)^2);
    end
end

% Shift DP values
voltageShiftDPInterp = voltageDPInterp(:,1);
for col = 2:(compOffsetDP + 1)
    for row = 1:size(voltageDPInterp,1)
        voltageShiftDPInterp(row,col) = ...
            voltageDPInterp(row,col)*cos(2*pi*50*voltageDPInterp(row,1)) - ...
            voltageDPInterp(row,col+compOffsetDP)*sin(2*pi*50*voltageDPInterp(row,1));
    end
end

% Calculate MSEs
mseDPInterp = sum((voltageRef - voltageShiftDPInterp).^2) / size(voltageRef,1)
mseEMTInterp = sum((voltageRef - voltageEMTInterp).^2) / size(voltageRef,1)

% Plot
figure2 = figure('Name',['DP EMT Interpolated Comparison ' num2str(voltageDP(2,1))],'NumberTitle','off');
axes2 = axes('Parent',figure2);
hold(axes2,'on');

voltageEMTInterpPlot = plot(voltageEMTInterp(:,1),voltageEMTInterp(:,plotNode), 'b--');
DPInterpplot = plot(voltageShiftDPInterp(:,1),voltageShiftDPInterp(:,plotNode), 'r-.');
DPabsInterpPlot = plot(voltageAbsDP(:,1),voltageAbsDP(:,plotNode), 'k-');
RefInterpsPlot = plot(voltageRef(:,1),voltageRef(:,plotNode), 'm:');
legend('EMT', 'DP shift', 'DP abs', 'Ref')
xlabel('time [s]')
ylabel('voltage [V]')

end

