% Author: Andres Acosta <pipeacosta@gmail.com>
% SPDX-FileCopyrightText: 2014-2024 Institute for Automation of Complex Power Systems, RWTH Aachen University
% SPDX-License-Identifier: Apache-2.0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clc;

warning('off','Control:analysis:LsimStartTime')
set(0, 'DefaultLineLineWidth', 1.5);

n_x2 = 1;
n_u2 = 1;
n_y2 = 1;

R_3 = 1;
C_2 = 1;

A_2 = -1/(R_3*C_2);
B_2 = 1/C_2;
C_2 = 1;
D_2 = 0;

x_2_0 = 2;
y_2_0 = C_2*x_2_0;

fprintf('Output value from S_2: %f\n', y_2_0);

% Configuration and connection
u = udp(DPSIM_HOST,                ...
    'LocalPort', 12009,             ...
    'RemotePort', 12008,            ...
    'Timeout', 10,                  ...
    'DatagramTerminateMode', 'on', ...
    'ByteOrder', 'littleEndian' ...
);
disp('UDP Receiver started');

fopen(u);
disp('UDP Socket bound');

num_values = 1;
t_s = 0.001;
t_f = 1;

t = 0:t_s:t_f;

ss_2 = ss(A_2, B_2, C_2, D_2);

u_2_v = zeros(1, length(t));
u_2_v(1) = 30; % This should be sent from the other subsystem
y_2_v = zeros(1, length(t));
y_2_v(1) = y_2_0;

% Send initial output
fwrite(u, y_2_0, 'float');

i = 2;

while i <= length(t)
    % Receive input from S_1
    [ u_2, c_u_2 ] = fread(u, num_values, 'float');
    
    fprintf('Received output value from S_1: %f\n', u_2(1));

    if ~isempty(u_2)
        u_2_v(i) = double(u_2(1));
    end
    
    [y_2,~,x_2] = lsim(ss_2, u_2_v(i-1)*ones(1,2), t(i-1:i), x_2_0);
    fprintf('Output value from S_2: %f\n', y_2(end));
    
%     fwrite(u, [0, single(y_2(end))], 'float');
    fwrite(u, y_2(end), 'float');
    
    x_2_0 = x_2(end);
    y_2_v(i) = y_2(end);
    i = i + 1;
end

figure();
plot(t, u_2_v);
grid on
xlabel('t [s]')
ylabel('$i_2$ [A]', 'Interpreter', 'Latex')

figure();
plot(t, y_2_v);
grid on
xlabel('t [s]')
ylabel('$v_2$ [V]', 'Interpreter', 'Latex')

fclose(u);
delete(u);
