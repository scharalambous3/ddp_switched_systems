function [t, x] = dyn_rollout_discrete(fun,x0,u, tRange)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global dt
x = zeros(size(x0,1),length(tRange));
x(:,1) = x0;
for i = 1:(length(tRange)-1)
    x(:,i+1) = x(:,i) + fun(0, x(:,i), u(:,i)) * dt; %+ dynamics.Fb(u_new(1,k), x(3,k)) * sqrt(dt) * sigma * randn;
end
t = tRange;
end