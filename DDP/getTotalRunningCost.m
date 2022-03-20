function [cost] = getTotalRunningCost(Q, R, horLength, xTraj, uTraj, xDesTraj, uDesTraj, P)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
cost = 0;
global dt
if nargin < 9
    P = zeros(size(R,1), size(Q,1));
end
for i =1:horLength
    xDes = xDesTraj(:,i);
    uDes = uDesTraj(:,i);
    x = xTraj(:,i);
    u = uTraj(:,i);
    
    xbar= x - xDes;
    ubar = u - uDes;
    cost = cost + (0.5 * xbar'*(Q*xbar) + 0.5 * ubar'*(R*ubar) + 0.5 * ubar'*(P*xbar))*dt;
    
end
    
end

