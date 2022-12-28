function [t, x] = dyn_rollout_discrete(fun,x0,u, tRange)
%dyn_rollout_discrete Integrate trajectory forward from a given initial
%condition and input sequence (discrete time)
global dt
x = zeros(size(x0,1),length(tRange));
x(:,1) = x0;
for i = 1:(length(tRange)-1)
    % First order Euler method
    x(:,i+1) = x(:,i) + fun(0, x(:,i), u(:,i)) * dt;
end
t = tRange;
end