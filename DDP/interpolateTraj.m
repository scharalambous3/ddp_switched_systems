function [x] = interpolateTraj(t,tRange, xTraj)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
delta_t = t - tRange;
signedVec = sign(delta_t);
indexSecond = find(signedVec < 0, 1, 'first');
if size(xTraj,1) > size(xTraj,2)
   xTraj = xTraj'; 
end
if (isempty(indexSecond))
   x = xTraj(:, end);
else
x = ((t-tRange(indexSecond-1))/(tRange(indexSecond)-tRange(indexSecond-1)))*(xTraj(:,indexSecond)-xTraj(:,indexSecond-1)) + xTraj(:,indexSecond-1);
end
end

