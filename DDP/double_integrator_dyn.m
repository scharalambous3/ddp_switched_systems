function [dxdt] = double_integrator_dyn(t, x, u, tRange)
%DOUBLE_INTEGRATOR_DYN Summary of this function goes here


%   Detailed explanation goes here
dxdt(1)= x(2);
if size(u,2) > 1
    u = interpolateTraj(t,tRange,u);
end
dxdt(2)= u;
dxdt = dxdt';
end

