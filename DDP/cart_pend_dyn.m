function [dxdt] = double_integrator_dyn(t, x, u, tRange)
%DOUBLE_INTEGRATOR_DYN Summary of this function goes here


%   Detailed explanation goes here
if size(u,2) > 1
    u = interpolateTraj(t,tRange,u);
end

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

Sx = sin(x(3));
Cx = cos(x(3));
D = m*L*L*(M+m*(1-Cx^2));

dxdt(1,1) = x(2);
dxdt(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
dxdt(3,1) = x(4);
dxdt(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u; % +.01*randn;
end

