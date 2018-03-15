% Compare voltage and current of c++ simulation with voltage and currents
% from PLECS simulation
clc
clear all
%% read PLECS results
Results_Reference= csvread('../../../vsa/Results/LoadChange/Simulink/Voltages_and_currents.csv');
l_Ref = length(Results_Reference);
figure(1)
hold off
plot(Results_Reference(:,1),Results_Reference(:,5))
hold on

%Calculate reference peak values for steady state and after load change
Peak_Ref_SS = max(Results_Reference(1:l_Ref/3,5));
Peak_Ref_LC = max(Results_Reference(l_Ref/3:2*l_Ref/3,5));



i = 1;

FileNameVector = ['SynGen_VBR_0.000050.csv';
    'SynGen_VBR_0.000100.csv';
    'SynGen_VBR_0.000150.csv';
    'SynGen_VBR_0.000200.csv';
    'SynGen_VBR_0.000250.csv';
    'SynGen_VBR_0.000300.csv';
    'SynGen_VBR_0.000350.csv';
    'SynGen_VBR_0.000400.csv';
    'SynGen_VBR_0.000450.csv';
    'SynGen_VBR_0.000500.csv';
    'SynGen_VBR_0.000550.csv';
    'SynGen_VBR_0.000600.csv';
    'SynGen_VBR_0.000650.csv';
    'SynGen_VBR_0.000700.csv';
    'SynGen_VBR_0.000750.csv';
    'SynGen_VBR_0.000800.csv';
    'SynGen_VBR_0.000850.csv';
    'SynGen_VBR_0.000900.csv';
    'SynGen_VBR_0.000950.csv';
    'SynGen_VBR_0.001000.csv'];


%% read results from c++ simulation
for dt = 0.000050: 0.000050: 0.001
   
    FileName = FileNameVector(i,1:23);
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
err_SS = sqrt(immse(CurrentVector_SS,Reference_SS));
disp('############ Error for VBR EMT model ###############');
disp(['############     dt= ',num2str(dt),'         ###############']);
disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

% Calculate maximum error and root mean squared error after load change
Dif_LC = abs(CurrentVector_LC - Reference_LC);
[MaxDif_LC,i1] = max(Dif_LC);
err_LC = sqrt(immse(CurrentVector_LC,Reference_LC));
disp(' ');
disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC/Peak_Ref_LC), ' %']);
disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC/Peak_Ref_LC), ' %']);
disp(' ');
disp(' ');

    ErrorVector_LC(i) = 100*err_LC/Peak_Ref_LC;
    dtVector(i) = dt;
    i = i + 1;
end
 
figure(2)
hold off
plotF1 = plot(dtVector,ErrorVector_LC);
hold on


 
%% Dq Model
 
figure(3)
hold off
plot(Results_Reference(:,1),Results_Reference(:,5))
hold on

i = 1;

FileNameVector = ['SynGen_Dq_0.000050.csv';
    'SynGen_Dq_0.000100.csv';
    'SynGen_Dq_0.000150.csv';
    'SynGen_Dq_0.000200.csv';
    'SynGen_Dq_0.000250.csv';
    'SynGen_Dq_0.000300.csv';
    'SynGen_Dq_0.000350.csv';
    'SynGen_Dq_0.000400.csv';
    'SynGen_Dq_0.000450.csv';
    'SynGen_Dq_0.000500.csv';
    'SynGen_Dq_0.000550.csv';
    'SynGen_Dq_0.000600.csv';
    'SynGen_Dq_0.000650.csv';
    'SynGen_Dq_0.000700.csv'];

%read results from c++ simulation
for dt = 0.00005: 0.00005: 0.0007
    FileName = FileNameVector(i,1:22);
    Path = strcat('../../../vsa/Results/LoadChange/DPsim/EMT/Dq/', FileName);
    Log_SynGen = csvread(Path,1);
    CurrentVector = Log_SynGen(:,1:4);
    
  %% Calculate and display error

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
    err_SS = sqrt(immse(CurrentVector_SS,Reference_SS));
    disp('############ Error for Dq EMT model ###############');
    disp(['############     dt= ',num2str(dt),'         ###############']);
    disp(['Maximum Error ia steady state: ', num2str(100*MaxDif_SS/Peak_Ref_SS), ' %']);
    disp(['Root Mean-squared error ia steady state: ', num2str(100*err_SS/Peak_Ref_SS), ' %']);

    % Calculate maximum error and root mean squared error after load change
    Dif_LC = abs(CurrentVector_LC - Reference_LC);
    [MaxDif_LC,i1] = max(Dif_LC);
    err_LC = sqrt(immse(CurrentVector_LC,Reference_LC));
    disp(' ');
    disp(['Maximum Error ia load change: ', num2str(100*MaxDif_LC/Peak_Ref_LC), ' %']);
    disp(['Root Mean-squared error ia load change: ', num2str(100*err_LC/Peak_Ref_LC), ' %']);
    disp(' ');
    disp(' ');

    ErrorVector_LC2(i) = 100*err_LC/Peak_Ref_LC;
    dtVector2(i) = dt;
    i = i + 1;
end

 figure(2)
 plotF2 = plot(dtVector2,ErrorVector_LC2);
 legend('Load Change Error - VBR','Load Change Error _ DQ')
 ylabel('Error');
 xlabel('Time Step');
 set(plotF2,'LineWidth',2);
 set(plotF1,'LineWidth',2);