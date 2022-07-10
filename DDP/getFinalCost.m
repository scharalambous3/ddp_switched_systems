function [finalCost] = getFinalCost(Qf, xTraj, xDesTraj)
%getFinalCost Return quadratic approximation of terminal cost

finalCost.value = 0.5 * (xTraj(:,end) - xDesTraj(:,end))'*Qf*(xTraj(:,end) - xDesTraj(:,end));
finalCost.dx = Qf*(xTraj(:,end) - xDesTraj(:,end));
finalCost.dxx = Qf;
end

