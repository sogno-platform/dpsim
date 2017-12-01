
% model with 2 nodes
fileName1='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExampleRXLine3_0.001.csv';

% circuit with R and L
fileName2='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExampleRXLine2_0.001.csv'

%model with 3 nodes
fileName3='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExampleRXLine_0.001.csv';

compareDpResults(fileName1,fileName2,2,3,'Model with 2 nodes x Resistor + Inductor');

compareDpResults(fileName3,fileName2,2,3,'Model with 3 nodes x Resistor + Inductor');
