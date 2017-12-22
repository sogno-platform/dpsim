% [ Td0_t, Td0_s, Td_t, Td_s, Ld_t, Ld_s, Tq0_t, Tq0_s, ...
%     Lq_t, Lq_s, Ld, Lq  ] = ...
%     FundamentalToStandard(0.15, 1.66, 0.165, 0.1713, 0.0284, 0.0006, 1.61, 0.7252, 0.125, 0.0062, 0.0237)

clear all

Ld = 1.81;
Ld_t = 0.3;
Td0_t = 8;
Lq = 1.76;
Lq_t = 0.65;
Tq0_t =1;
Ll = 0.15;
Ld_s = 0.23;
Td0_s = 0.03;
Ra = 0.003;
Lq_s = 0.25;
Tq0_s = 0.07;

[ Lad, Laq, Lfd, L1q, L1d, L2q, Rfd, R1d, R1q, R2q ] = StandardToFundamental( Ld, Lq, Ld_t, Lq_t, Ld_s, Lq_s, Ll, ...
    Ra, Td0_t, Tq0_t, Td0_s, Tq0_s)
