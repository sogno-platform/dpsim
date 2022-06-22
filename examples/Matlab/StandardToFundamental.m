function [ Lad, Laq, Lfd, L1q, L1d, L2q, Rfd, R1d, R1q, R2q ] = StandardToFundamental( Ld, Lq, Ld_t, Lq_t, Ld_s, Lq_s, Ll, ...
    Ra, Td0_t, Tq0_t, Td0_s, Tq0_s)
%StandardToFundamental converts Standard machine parameters to fundamental
% parameters
%   Detailed explanation goes here

Lad = Ld - Ll;
Laq = Lq - Ll;

Lfd = Lad*(Ld_t - Ll)/(Lad-Ld_t+Ll);
L1q = Laq*(Lq_t - Ll)/(Laq-Lq_t+Ll);

L1d = Lad*Lfd*(Ld_s-Ll)/(Lfd*Lad-(Lad+Lfd)*(Ld_s-Ll));
L2q = Laq*L1q*(Lq_s-Ll)/(L1q*Laq-(Laq+L1q)*(Lq_s-Ll));

Rfd = (Lad + Lfd)/(Td0_t*377);
R1d = (1/(Td0_s*377))*(L1d + Lad*Lfd/(Lad+Lfd));
R1q = (Laq + L1q)/(Tq0_t*377);
R2q = (1/(Tq0_s*377))*(L2q + Laq*L1q/(Laq+L1q));
end

