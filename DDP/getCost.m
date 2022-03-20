function [cost] = getCost(Q, R, i, x, u, xDesTraj, uDesTraj, P)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global dt
if nargin < 9
    P = zeros(size(R,1), size(Q,1));
end
% xDes = interpolateTraj(t,tRange,xDesTraj);
% uDes = interpolateTraj(t,tRange,uDesTraj);
xDes = xDesTraj(:, i);
uDes = uDesTraj(:, i);

xbar=x - xDes;
ubar = u - uDes;
cost.value = (0.5 * xbar'*(Q*xbar) + 0.5 * ubar'*(R*ubar) + 0.5 * ubar'*(P*xbar))*dt;
cost.xdeviation = xbar;
cost.udeviation = ubar;
%Quadratic approximation of cost function with respect to input and state
cost.dx = (Q*xbar + P'*ubar)*dt;
cost.du = (R*ubar + P*xbar)*dt;
cost.dxx = Q*dt;
cost.duu = R*dt;
cost.dux = P*dt;
end

