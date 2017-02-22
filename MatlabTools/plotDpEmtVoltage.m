function [ ] = plotDpEmtVoltage(filenameVoltageDP, filenameVoltageEMT, plotNode)

% Increment node to skip time column
plotNode = plotNode + 1

% Read values from CSV files
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

%% Calculate MSE
mseDP = sum((voltageEMT - voltageShiftDP).^2) / size(voltageEMT,1)

%% Plot
figure1 = figure('Name', ['DP EMT Comparison ' num2str(voltageDP(2,1))],'NumberTitle','off');
axes1 = axes('Parent',figure1);
hold(axes1,'on');

EMTplot = plot(voltageEMT(:,1),voltageEMT(:,plotNode), 'b--');
DPplot = plot(voltageShiftDP(:,1),voltageShiftDP(:,plotNode), 'r-.');
DPabsPlot = plot(voltageAbsDP(:,1),voltageAbsDP(:,plotNode), 'k-');

legend('EMT', 'DP shift', 'DP abs')
xlabel('time [s]')
ylabel('voltage [V]')

end