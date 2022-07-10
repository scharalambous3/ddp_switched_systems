function [cost] = getCost(Q, R, i, x, u, xDesTraj, uDesTraj, P)
%getCost Return quadratic approximation of stage cost at a specific stage,
%i, for specific state and input values
global dt
if nargin < 9
    P = zeros(size(R,1), size(Q,1));
end

%Desired values for state and input
xDes = xDesTraj(:, i);
uDes = uDesTraj(:, i);

%State and input deviation
xbar=x - xDes;
ubar = u - uDes;

%Quadratic approximation of cost function with respect to input and state
cost.value = (0.5 * xbar'*(Q*xbar) + 0.5 * ubar'*(R*ubar) + 0.5 * ubar'*(P*xbar))*dt;
cost.xdeviation = xbar;
cost.udeviation = ubar;
cost.dx = (Q*xbar + P'*ubar)*dt;
cost.du = (R*ubar + P*xbar)*dt;
cost.dxx = Q*dt;
cost.duu = R*dt;
cost.dux = P*dt;
end

