function [dyn] = cart_pend_dynMatrices(t, x, u, tRange)
%DOUBLE_INTEGRATOR_DYN Summary of this function goes here
% TIme variable inputs are redundant as dynamcs are time-invariant
% dxdt(1,1) = x(2)
% dxdt(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
% dxdt(3,1) = x(4);
% dxdt(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u; % +.01*randn;
% Equations from: https://github.com/bertozzijr/Control_Bootcamp_S_Brunton
if nargin < 1
   t=0;
   x=[];
   u=0;
   tRange=[];
end

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

%   Detailed explanation goes here
dyn.fx=zeros(4,4);
dyn.fu=zeros(4,1);
dyn.fxx=zeros(4,4,4);
dyn.fux=zeros(1,4,4);
dyn.fuu=zeros(1,1,4);



Sx = sin(x(3));
Cx = cos(x(3));
D = m*L*L*(M+m*(1-Cx^2));

dyn.fx(1, 2) = 1;
dyn.fx(2, 2) = -(1/D)*m*L^2*d;
dyn.fx(2, 3) = (1/D)*(-m^2*L^2*g*(-Sx*Sx+Cx*Cx) + m*L^2*(m*L*x(4)^2*Cx));
dyn.fx(2, 4) = (1/D)*(m*L^2*(m*L*2*x(4)*Sx));
dyn.fx(3, 4) = 1;
dyn.fx(4, 2) = (1/D)*(- m*L*Cx*(- d));
dyn.fx(4, 3) = (1/D)*((m+M)*m*g*L*Cx - m*L*(m*L*x(4)^2*(-Sx*Sx+Cx*Cx)));
dyn.fx(4, 4) = (1/D)*(- m*L*Cx*(m*L*2*x(4)*Sx));

dyn.fu(2, 1) = m*L*L*(1/D);
dyn.fu(4, 1) = - m*L*Cx*(1/D);


% 
% dxdt(1,1) = x(2);
% dxdt(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
% dxdt(3,1) = x(4);
% dxdt(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u; % +.01*randn;
end

