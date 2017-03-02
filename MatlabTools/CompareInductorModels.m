function [ ] = CompareInductorModels(filenameVoltageInductor1, filenameVoltageInductor2, plotNode)

% Increment node to skip time column
plotNode = plotNode + 1

% Read values from CSV files
voltageInductor1 = csvread(filenameVoltageInductor1);
compOffsetDP = (size(voltageInductor1,2) - 1) / 2;
voltageInductor2 = csvread(filenameVoltageInductor2);

% Calculate DP absolute value
voltageAbs1 = voltageInductor1(:,1);
for col = 2:( compOffsetDP + 1 )
    for row = 1:size(voltageInductor1,1)
        voltageAbs1(row,col) = sqrt(voltageInductor1(row,col)^2 + ...
            voltageInductor1(row,col+compOffsetDP)^2);
    end
end
voltageAbs2 = voltageInductor2(:,1);
for col = 2:( compOffsetDP + 1 )
    for row = 1:size(voltageInductor2,1)
        voltageAbs2(row,col) = sqrt(voltageInductor2(row,col)^2 + ...
            voltageInductor2(row,col+compOffsetDP)^2);
    end
end


% Shift DP values
voltageShiftDP1 = voltageInductor1(:,1);
for col = 2:(compOffsetDP + 1)
    for row = 1:size(voltageInductor1,1)
        voltageShiftDP1(row,col) = voltageInductor1(row,col)*cos(2*pi*50*voltageInductor1(row,1)) - ...
            voltageInductor1(row,col+compOffsetDP)*sin(2*pi*50*voltageInductor1(row,1));
    end
end
voltageShiftDP2 = voltageInductor2(:,1);
for col = 2:(compOffsetDP + 1)
    for row = 1:size(voltageInductor2,1)
        voltageShiftDP2(row,col) = voltageInductor2(row,col)*cos(2*pi*50*voltageInductor2(row,1)) - ...
            voltageInductor2(row,col+compOffsetDP)*sin(2*pi*50*voltageInductor2(row,1));
    end
end


%% Plot



DPplot = plot(voltageShiftDP1(:,1),voltageShiftDP1(:,plotNode), 'g--o');
hold on
DPabsPlot = plot(voltageAbs1(:,1),voltageAbs1(:,plotNode), 'b--o');
DPplot2 = plot(voltageShiftDP2(:,1),voltageShiftDP2(:,plotNode), 'y');

DPabsPlot2 = plot(voltageAbs2(:,1),voltageAbs2(:,plotNode), 'r');

legend('Shift1', 'Abs 1', 'Shift2','Abs 2')
xlabel('time [s]')
ylabel('voltage [V]')

end