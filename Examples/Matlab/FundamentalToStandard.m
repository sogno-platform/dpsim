function [ Td0_t, Td0_s, Td_t, Td_s, Ld_t, Ld_s, Tq0_t, Tq0_s,...
    Lq_t, Lq_s, Ld, Lq  ] = ...
    FundamentalToStandard(Ll, Lad, Lfd, L1d, R1d, Rfd,...
    Laq, L1q, L2q, R1q, R2q)
%FundamentalToStandard Transform fundamental machine parameters to standard
%parameters
%   The calculations are based on the classical definition. 
% Equations used are from Kundur p. 144 - 147

% ---- Relationship between parameters used in DPSim and parameters
% defined in Kundur: 
% Lad = Lmd
% L1d = Llkd
% Lfd = Llfd
% Laq = Lmq
% L1q = Llkq1
% L2q = Llkq2

%% Calculation of standard parameters

% D-axis inductance
Ld = Lad + Ll;
Lq = Laq + Ll;
% Time constants of d-axis
Td0_t = (Lad + Lfd)/Rfd;
Td0_s = (1/R1d)*(L1d + (Lad*Lfd)/(Lad + Lfd));
Td_t = (1/Rfd)*(Lfd + (Lad*Ll)/(Lad + Ll));
Td_s = (1/R1d)*(L1d + (Lad*Lfd*Ll)/(Lad*Ll + Lad*Lfd + Lfd*Ll));

Td0_t = Td0_t/377;
Td0_s = Td0_s/377;
Td_t = Td_t/377;
Td_s = Td_s/377;

% Inductances of d-axis
Ld_s = Ld*(Td_t*Td_s/(Td0_t*Td0_s));
Ld_t = Ld*(Td_t/Td0_t);

% Time constants of q-axis
Tq0_t = (Laq + L1q)/R1q;
Tq0_s = (1/R2q)*(L2q + (Laq*L1q)/(Laq + L1q));
% Tq_t = (1/R1q)*(L1q + (Laq*Ll)/(Laq + Ll));
% Tq_s = (1/R2q)*(L2q + (Laq*L1q*Ll)/(Laq*Ll + Laq*L1q + L1q*Ll));

Tq0_t = Tq0_t/377;
Tq0_s = Tq0_s/377;
% Tq_t = Tq_t/377;
% Tq_s = Tq_s/377;

% Inductances of q-axis
Lq_s = Ll + Laq*L1q*L2q/(Laq*L1q + Laq*L2q + L1q*L2q);
Lq_t = Ll + Laq*L1q/(Laq+L1q);

end

