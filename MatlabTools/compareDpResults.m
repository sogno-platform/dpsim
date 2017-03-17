function [ ] = compareDpResults(filenameVoltageDP1, filenameVoltageDP2, node1, node2)

% Increment node to skip time column
plotNode1 = node1 + 1;
plotNode2 = node2 + 1;
% Read values from CSV files
voltageDP1 = csvread(filenameVoltageDP1);
compOffsetDP1 = (size(voltageDP1,2) - 1) / 2;

voltageDP2 = csvread(filenameVoltageDP2);
compOffsetDP2 = (size(voltageDP2,2) - 1) / 2;

% Calculate DP absolute value
voltageAbsDP1 = voltageDP1(:,1);
for col = 2:( compOffsetDP1 + 1 )
    for row = 1:size(voltageDP1,1)
        voltageAbsDP1(row,col) = sqrt(voltageDP1(row,col)^2 + ...
            voltageDP1(row,col+compOffsetDP1)^2);
    end
end
voltageAbsDP2 = voltageDP2(:,1);
for col = 2:( compOffsetDP2 + 1 )
    for row = 1:size(voltageDP2,1)
        voltageAbsDP2(row,col) = sqrt(voltageDP2(row,col)^2 + ...
            voltageDP2(row,col+compOffsetDP2)^2);
    end
end

% Shift DP values
voltageShiftDP1 = voltageDP1(:,1);
for col = 2:(compOffsetDP1 + 1)
    for row = 1:size(voltageDP1,1)
        voltageShiftDP1(row,col) = voltageDP1(row,col)*cos(2*pi*50*voltageDP1(row,1)) - ...
            voltageDP1(row,col+compOffsetDP1)*sin(2*pi*50*voltageDP1(row,1));
    end
end
voltageShiftDP2 = voltageDP2(:,1);
for col = 2:(compOffsetDP2 + 1)
    for row = 1:size(voltageDP2,1)
        voltageShiftDP2(row,col) = voltageDP2(row,col)*cos(2*pi*50*voltageDP2(row,1)) - ...
            voltageDP2(row,col+compOffsetDP2)*sin(2*pi*50*voltageDP2(row,1));
    end
end

%% Calculate MSE
%mseDP = sum((voltageEMT - voltageShiftDP).^2) / size(voltageEMT,1)

%% Plot
figure1 = figure('Name', 'Comparison','NumberTitle','off');
axes1 = axes('Parent',figure1);
hold(axes1,'on');

DP1plot = plot(voltageShiftDP1(:,1),voltageShiftDP1(:,plotNode1), 'r-.');
DP1absPlot = plot(voltageAbsDP1(:,1),voltageAbsDP1(:,plotNode1), 'k-');
DP2plot = plot(voltageShiftDP2(:,1),voltageShiftDP2(:,plotNode2), 'g-.');
DP2absPlot = plot(voltageAbsDP2(:,1),voltageAbsDP2(:,plotNode2), 'b-');

legend('DP1 shift', 'DP1 abs', 'DP2 shift', 'DP2 abs')
xlabel('time [s]')
ylabel('voltage [V]')

end