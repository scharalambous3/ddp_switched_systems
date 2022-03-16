function [t,x] = dyn_rollout(fun,x0,u, tRange)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

[t x]=ode45(@(t, x0) fun(t,x0,u, tRange),tRange,x0, u');
x = x';
end