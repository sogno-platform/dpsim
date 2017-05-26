% circuit with R, L and C
fileName1='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExamplePiLine_0.001.csv'

%model with 3 nodes
fileName2='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExamplePiLine2_0.001.csv';

compareDpResults(fileName1,fileName2,1,1,'Model with 3 nodes x Resistor + Inductor');

