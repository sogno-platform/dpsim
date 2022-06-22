% Calculate root mean squared error for the current in the interval 0.1 and
% 0.2s and plot error in function of time step

clc
clear all

%% read Reference results
Results_Reference= csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Results_Reference(1:l_Ref/3,5));
Peak_Ref_LC = max(Results_Reference(l_Ref/3:2*l_Ref/3,5));


%% VBR model
i = 1;

% List of files to be used
FileNameVector = ['SynGen_EMT_VBR_0.000050.csv';
    'SynGen_EMT_VBR_0.000100.csv';
    'SynGen_EMT_VBR_0.000200.csv';
    'SynGen_EMT_VBR_0.000300.csv';
    'SynGen_EMT_VBR_0.000400.csv';
    'SynGen_EMT_VBR_0.000500.csv';
    'SynGen_EMT_VBR_0.000600.csv';
    'SynGen_EMT_VBR_0.000700.csv';
    'SynGen_EMT_VBR_0.000800.csv';
    'SynGen_EMT_VBR_0.000900.csv';
    'SynGen_EMT_VBR_0.001000.csv';
    'SynGen_EMT_VBR_0.002000.csv';
    'SynGen_EMT_VBR_0.003000.csv';
    'SynGen_EMT_VBR_0.004000.csv';
    'SynGen_EMT_VBR_0.005000.csv';
    'SynGen_EMT_VBR_0.006000.csv';
    'SynGen_EMT_VBR_0.007000.csv';
    'SynGen_EMT_VBR_0.008000.csv';
    'SynGen_EMT_VBR_0.009000.csv';
    'SynGen_EMT_VBR_0.010000.csv'];

dt = 0.000050;
%% read results from c++ simulation
while dt <= 0.011000
   
    FileName = FileNameVector(i,1:27);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/EMT/VBR/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:4);
    
 %% Calculate and display error
%Cut Current and Voltage vector to get steady state results
l=length(CurrentVector);
l_new=round(1/3*l);

CurrentVector_SS = -CurrentVector(1:l_new,2);
CurrentVector_LC = -CurrentVector(l_new:2*l_new,2);

CurrentReference_reduced = zeros(l,2);
    
if l == l_Ref
    CurrentReference_reduced(:,1) = Results_Reference(:,1);
    CurrentReference_reduced(:,2) = Results_Reference(:,5);
else
    s = round(dt/5e-5);
    n = 1;
    for m = 1:s:l_Ref
    CurrentReference_reduced(n,1) = Results_Reference(m,1);
    CurrentReference_reduced(n,2) = Results_Reference(m,5);
    n = n+1;
    end
end  
 
%Reference current in Steady state and after load change
Reference_SS = CurrentReference_reduced(1:l_new,2);
Reference_LC = CurrentReference_reduced(l_new:2*l_new,2);



% Calculate maximum error and root mean squared error for steady state
Dif_SS = abs(CurrentVector_SS - Reference_SS);
[MaxDif_SS,i1] = max(Dif_SS);
%err_SS = sqrt(immse(CurrentVector_SS,Reference_SS));
err_SS = sqrt( mean((CurrentVector_SS - Reference_SS).^2))
disp('############ Error for VBR EMT model ###############');
disp(['############     dt= ',num2str(dt),'         ###############']);
disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

% Calculate maximum error and root mean squared error after load change
Dif_LC = abs(CurrentVector_LC - Reference_LC);
[MaxDif_LC,i1] = max(Dif_LC);
%err_LC = sqrt(immse(CurrentVector_LC,Reference_LC));
err_LC = sqrt( mean((CurrentVector_LC - Reference_LC).^2))
disp(' ');
disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC/Peak_Ref_LC), ' %']);
disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC/Peak_Ref_LC), ' %']);
disp(' ');
disp(' ');

    ErrorVector_VBR(i) = 100*err_LC/Peak_Ref_LC;
    dtVectorVBR(i) = dt;
    i = i + 1;
    
    if dt >= 0.001
        dt = dt + 0.001;
    elseif dt >= 0.0001
        dt = dt + 0.0001;
    else
        dt = dt + 0.000050;
    end
end
 

 
%% Dq Model

i = 1;

% List of files to be used
FileNameVector = ['SynGen_EMT_Dq_0.000050.csv';
    'SynGen_EMT_Dq_0.000100.csv';
    'SynGen_EMT_Dq_0.000200.csv';
    'SynGen_EMT_Dq_0.000300.csv';
    'SynGen_EMT_Dq_0.000400.csv';
    'SynGen_EMT_Dq_0.000500.csv';
    'SynGen_EMT_Dq_0.000600.csv';
    'SynGen_EMT_Dq_0.000700.csv'];

dt = 0.000050;
while dt <= 0.000800
      
    FileName = FileNameVector(i,1:26);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/EMT/Dq/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:4);
    
    l=length(CurrentVector);
    l_new=round(1/3*l);

    CurrentVector_SS = CurrentVector(1:l_new,2);
    CurrentVector_LC = CurrentVector(l_new:2*l_new,2);

    CurrentReference_reduced = zeros(l,2);
    
    if l == l_Ref
     CurrentReference_reduced(:,1) = Results_Reference(:,1);
     CurrentReference_reduced(:,2) = Results_Reference(:,5);
    else
        s = round(dt/5e-5);
        n = 1;
        for m = 1:s:l_Ref
            CurrentReference_reduced(n,1) = Results_Reference(m,1);
            CurrentReference_reduced(n,2) = Results_Reference(m,5);
            n = n+1;
        end
    end  
 
    %Reference current in Steady state and after load change
    Reference_SS = CurrentReference_reduced(1:l_new,2);
    Reference_LC = CurrentReference_reduced(l_new:2*l_new,2);

    % Calculate maximum error and root mean squared error for steady state
    Dif_SS = abs(CurrentVector_SS - Reference_SS);
    [MaxDif_SS,i1] = max(Dif_SS);
    %err_SS = sqrt(immse(CurrentVector_SS,Reference_SS));
    err_SS = sqrt( mean((CurrentVector_SS - Reference_SS).^2))
    disp('############ Error for Dq EMT model ###############');
    disp(['############     dt= ',num2str(dt),'         ###############']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC = abs(CurrentVector_LC - Reference_LC);
    [MaxDif_LC,i1] = max(Dif_LC);
    %err_LC = sqrt(immse(CurrentVector_LC,Reference_LC));
    err_LC = sqrt( mean((CurrentVector_LC - Reference_LC).^2))
    disp(' ');
    disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');

    ErrorVector_dq(i) = 100*err_LC/Peak_Ref_LC;
    dtVectordq(i) = dt;
    i = i + 1;
    
    if dt >= 0.0001
        dt = dt + 0.0001;
    else
        dt = dt + 0.000050;
    end
end

%% DP results

i = 1;

FileNameVector = ['SynGen_DP_VBR_0.000050.csv';
    'SynGen_DP_VBR_0.000100.csv';
    'SynGen_DP_VBR_0.000200.csv';
    'SynGen_DP_VBR_0.000300.csv';
    'SynGen_DP_VBR_0.000400.csv';
    'SynGen_DP_VBR_0.000500.csv';
    'SynGen_DP_VBR_0.000600.csv';
    'SynGen_DP_VBR_0.000700.csv';
    'SynGen_DP_VBR_0.000800.csv';
    'SynGen_DP_VBR_0.000900.csv';
    'SynGen_DP_VBR_0.001000.csv';
    'SynGen_DP_VBR_0.002000.csv';
    'SynGen_DP_VBR_0.003000.csv';
    'SynGen_DP_VBR_0.004000.csv';
    'SynGen_DP_VBR_0.005000.csv';
    'SynGen_DP_VBR_0.006000.csv';
    'SynGen_DP_VBR_0.007000.csv';
    'SynGen_DP_VBR_0.008000.csv';
    'SynGen_DP_VBR_0.009000.csv';
    'SynGen_DP_VBR_0.010000.csv'];

dt = 0.000050;

while dt <= 0.011000
    FileName = FileNameVector(i,1:26);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/DP/VBR/', FileName);
    Log_SynGen = csvread(Path,1);
    currentDP = Log_SynGen(:,1:7);
    compOffsetDP = (size(currentDP,2) - 1) / 2;

    currentShiftDP = currentDP(:,1);
    col = 2;
    for row = 1:size(currentDP,1)
        currentShiftDP(row,col) = currentDP(row,col)*cos(2*pi*60*currentDP(row,1)) - ...
            currentDP(row,col+compOffsetDP)*sin(2*pi*60*currentDP(row,1));
    end

    
    l_DP=length(currentShiftDP);
    l_new_DP=round(1/3*l_DP);

    CurrentVector_SS_DP = -currentShiftDP(1:l_new_DP,2);
    CurrentVector_LC_DP = -currentShiftDP(l_new_DP:2*l_new_DP,2);

    CurrentReference_reduced = zeros(l_DP,2);
    
    if l_DP == l_Ref
        CurrentReference_reduced(:,1) = Results_Reference(:,1);
        CurrentReference_reduced(:,2) = Results_Reference(:,5);
    else
        s = round(dt/5e-5);
        n = 1;
        for m = 1:s:l_Ref
            CurrentReference_reduced(n,1) = Results_Reference(m,1);
            CurrentReference_reduced(n,2) = Results_Reference(m,5);
            n = n+1;
        end
    end  
 
    %Reference current in Steady state and after load change
    Reference_SS = CurrentReference_reduced(1:l_new_DP,2);
    Reference_LC = CurrentReference_reduced(l_new_DP:2*l_new_DP,2);

    % Calculate maximum error and root mean squared error for steady state
    Dif_SS_DP = abs(CurrentVector_SS_DP - Reference_SS);
    [MaxDif_SS_DP,i1] = max(Dif_SS_DP);
    %err_SS_DP = sqrt(immse(CurrentVector_SS_DP,Reference_SS));
    err_SS_DP = sqrt( mean((CurrentVector_SS_DP - Reference_SS).^2))
    disp('############ Error for VBR DP model ###############');
    disp(['############     dt= ',num2str(dt),'         ###############']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS_DP/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS_DP/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC_DP = abs(CurrentVector_LC_DP - Reference_LC);
    [MaxDif_LC_DP,i1] = max(Dif_LC_DP);
    %err_LC_DP = sqrt(immse(CurrentVector_LC_DP,Reference_LC));
    err_LC_DP = sqrt( mean((CurrentVector_LC_DP - Reference_LC).^2))
    disp(' ');
    disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC_DP/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC_DP/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');

    ErrorVector_VBRDP(i) = 100*err_LC_DP/Peak_Ref_LC;
    dtVectorVBRDP(i) = dt;
    i = i + 1;
    
    if dt >= 0.001
        dt = dt + 0.001;
    elseif dt >= 0.0001
        dt = dt + 0.0001;
    else
        dt = dt + 0.000050;
    end
end



%% Dq Model

i = 1;

FileNameVector = ['SynGen_DP_Dq_0.000050.csv';
    'SynGen_DP_Dq_0.000100.csv';
    'SynGen_DP_Dq_0.000200.csv';
    'SynGen_DP_Dq_0.000300.csv';
    'SynGen_DP_Dq_0.000400.csv';
    'SynGen_DP_Dq_0.000500.csv';
    'SynGen_DP_Dq_0.000600.csv';
    'SynGen_DP_Dq_0.000700.csv'];

dt = 0.000050;

while dt < 0.000750
    FileName = FileNameVector(i,1:25);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/DP/Dq/', FileName);
    Log_SynGen = csvread(Path,1);
    currentDP = Log_SynGen(:,1:7);
    compOffsetDP = (size(currentDP,2) - 1) / 2;
    
    currentShiftDP = currentDP(:,1);
    col = 2;
    for row = 1:size(currentDP,1)
        currentShiftDP(row,col) = currentDP(row,col)*cos(2*pi*60*currentDP(row,1)) - ...
            currentDP(row,col+compOffsetDP)*sin(2*pi*60*currentDP(row,1));
    end

    
    l_DP=length(currentShiftDP);
    l_new_DP=round(1/3*l_DP);

    CurrentVector_SS_DP = currentShiftDP(1:l_new_DP,2);
    CurrentVector_LC_DP = currentShiftDP(l_new_DP:2*l_new_DP,2);

    CurrentReference_reduced = zeros(l_DP,2);
    
    if l_DP == l_Ref
        CurrentReference_reduced(:,1) = Results_Reference(:,1);
        CurrentReference_reduced(:,2) = Results_Reference(:,5);
    else
        s = round(dt/5e-5);
        n = 1;
        for m = 1:s:l_Ref
            CurrentReference_reduced(n,1) = Results_Reference(m,1);
            CurrentReference_reduced(n,2) = Results_Reference(m,5);
            n = n+1;
        end
    end  
 
    %Reference current in Steady state and after load change
    Reference_SS = CurrentReference_reduced(1:l_new_DP,2);
    Reference_LC = CurrentReference_reduced(l_new_DP:2*l_new_DP,2);

    % Calculate maximum error and root mean squared error for steady state
    Dif_SS_DP = abs(CurrentVector_SS_DP - Reference_SS);
    [MaxDif_SS_DP,i1] = max(Dif_SS_DP);
    %err_SS_DP = sqrt(immse(CurrentVector_SS_DP,Reference_SS));
    err_SS_DP = sqrt( mean((CurrentVector_SS_DP - Reference_SS).^2))
    disp('############ Error for Dq DP model ###############');
    disp(['############     dt= ',num2str(dt),'         ###############']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS_DP/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS_DP/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC_DP = abs(CurrentVector_LC_DP - Reference_LC);
    [MaxDif_LC_DP,i1] = max(Dif_LC_DP);
    %err_LC_DP = sqrt(immse(CurrentVector_LC_DP,Reference_LC));
    err_LC_DP = sqrt( mean((CurrentVector_LC_DP - Reference_LC).^2))
    disp(' ');
    disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC_DP/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC_DP/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');
    ErrorVector_dqDP(i) = 100*err_LC_DP/Peak_Ref_LC;
    dtVectordqDP(i) = dt;
    i = i + 1;
    
        if dt >= 0.0001
        dt = dt + 0.0001;
    else
        dt = dt + 0.000050;
    end
end

% plot error in function of time step
h1 = figure(1);
hold off
plot1 = plot(dtVectorVBR,ErrorVector_VBR);
hold on
plot2 = plot(dtVectorVBRDP,ErrorVector_VBRDP);
[maxError,ie] = max(ErrorVector_VBRDP);
plotp = plot(dtVectorVBRDP(ie),ErrorVector_VBRDP(ie),'r*');
strmax = ['Maximum Error = ',num2str(maxError),' %'];
text(dtVectorVBRDP(ie),ErrorVector_VBRDP(ie)+ 0.7,strmax,'HorizontalAlignment','right','FontSize',12);
legend({'EMT VBR','DP VBR'},'FontSize',12)
ylabel('Error [%]','FontSize',12);
xlabel('Time Step [s]','FontSize',12);
 
set(plot1,'LineWidth',2);
set(plot2,'LineWidth',2);
set(plotp,'LineWidth',2);

set(h1,'Units','centimeters');
set(h1,'pos',[5 5 22 13])
pos = get(h1,'Position');
set(h1,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
ylim([0 20]);

h2 = figure(2);
hold off
plot3 = plot(dtVectorVBR(1,1:11),ErrorVector_VBR(1,1:11));
hold on
plot4 = plot(dtVectordq,ErrorVector_dq);
legend({'EMT VBR','EMT Classical'},'FontSize',12)
ylabel('Error [%]','FontSize',12);
xlabel('Time Step [s]','FontSize',12);
ylim([0 16]);
 
set(plot3,'LineWidth',2);
set(plot4,'LineWidth',2);

set(h2,'Units','centimeters');
set(h2,'pos',[5 5 22 13])
pos = get(h2,'Position');
set(h2,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])


h3 = figure(3);
hold off
plot3 = plot(dtVectorVBRDP(1,1:11),ErrorVector_VBRDP(1,1:11));
hold on
plot4 = plot(dtVectordqDP,ErrorVector_dqDP);
legend({'DP VBR','DP Classical'},'FontSize',12)
ylabel('Error [%]','FontSize',12);
xlabel('Time Step [s]','FontSize',12);
ylim([0 2]);
 
set(plot3,'LineWidth',2);
set(plot4,'LineWidth',2);

set(h3,'Units','centimeters');
set(h3,'pos',[5 5 22 13])
pos = get(h3,'Position');
set(h3,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
