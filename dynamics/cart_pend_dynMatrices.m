function [dyn] = cart_pend_dynMatrices(t, x, u)
%cart_pend_dynMatrices Return quadratic approximation of cart pendulum
%dynamics about specific state and input
global fxFunc fuFunc;
% Time variable inputs are redundant as dynamics are time-invariant
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

dyn.fx=zeros(4,4);
dyn.fu=zeros(4,1);
dyn.fxx=zeros(4,4,4);
dyn.fux=zeros(1,4,4);
dyn.fuu=zeros(1,1,4);

dyn.fx = fxFunc(x(3),x(4),u);
dyn.fu = fuFunc(x(3));

end

