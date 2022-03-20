function [dxdt] = cart_pend_dyn(t, x, u)
%DOUBLE_INTEGRATOR_DYN Summary of this function goes here


%   Detailed explanation goes here
% if size(u,2) > 1
%     u = interpolateTraj(t,tRange,u);
% end

m = 0.01;
M = 1;
L = 0.25;
g = -9.8;

Sx = sin(x(3));
Cx = cos(x(3));
D = m*L*L*(M+m*(1-Cx^2));

% dxdt(1,1) = x(2);
% dxdt(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
% dxdt(3,1) = x(4);
% dxdt(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u; % +.01*randn;
xVar = x(1);
xdot = x(2);
theta = x(3);
thetadot = x(4);
uVar = u;

dxdt(1,1) = xdot;
dxdt(2,1) = (uVar + m*sin(theta)*(L*thetadot^2 - g * cos(theta)))/(M+m*sin(theta)^2);
dxdt(3,1) = thetadot;
dxdt(4,1) = (-uVar*cos(theta) - m*L*thetadot^2*sin(theta)*cos(theta)+(M+m)*g*sin(theta))/(L*(M+m*sin(theta)^2));

end

