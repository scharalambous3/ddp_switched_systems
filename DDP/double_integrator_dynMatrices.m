function [dyn] = double_integrator_dynMatrices(t, x, u)
%DOUBLE_INTEGRATOR_DYN Summary of this function goes here
if nargin < 1
   t=0;
   x=[];
   u=0;
   tRange=[];
end

%   Detailed explanation goes here
dyn.fx=[0,1;0,0];
dyn.fu=[0;1];
dyn.fxx=zeros(2,2,2);
dyn.fux=zeros(1,2,2);
dyn.fuu=zeros(1,1,2);
end

