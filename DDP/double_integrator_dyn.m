function [dxdt] = double_integrator_dyn(t, x, u)
%double_integrator_dyn Compute flow map of double integrator dynamics

dxdt(1)= x(2);
% if size(u,2) > 1
%     u = interpolateTraj(t,tRange,u);
% end
dxdt(2)= u;
dxdt = dxdt';
end

