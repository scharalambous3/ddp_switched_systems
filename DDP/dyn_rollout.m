function [t,x] = dyn_rollout(fun,x0,u, tRange)
%dyn_rollout Integrate trajectory forward from a given initial condition
%and input sequence (continuous time)

[t, x]=ode45(@(t, x0) fun(t,x0,u, tRange),tRange,x0, u');
x = x';
end