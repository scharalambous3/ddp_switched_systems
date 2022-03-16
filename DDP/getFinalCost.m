function [finalCost] = getFinalCost(Qf, xTraj, xDesTraj)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

finalCost.value = 0.5 * (xTraj(:,end) - xDesTraj(:,end))'*Qf*(xTraj(:,end) - xDesTraj(:,end));
finalCost.dx = Qf*(xTraj(:,end) - xDesTraj(:,end));
finalCost.dxx = Qf;
end

