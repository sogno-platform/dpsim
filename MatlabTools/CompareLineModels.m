
fileName1='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExampleRXLine3_0.001.csv';
fileName2='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExampleRXLine2_0.001.csv'
compareDpResults(fileName1,fileName2,2,3,'Model with 2 nodes x Resistor + Inductor');

fileName3='../VisualStudio/DPsimVS2015/Logs/LeftVectorLog_SimulationExampleRXLine_0.001.csv';
compareDpResults(fileName3,fileName2,2,3,'Model with 3 nodes x Resistor + Inductor');
