function [dxdt] = cart_pend_dyn(t, x, u)
%cart_pend_dyn Compute flow map of cart pendulum dynamics

m = 0.01;
M = 1;
L = 0.25;
g = -9.8;

Sx = sin(x(3));
Cx = cos(x(3));
D = m*L*L*(M+m*(1-Cx^2));

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

