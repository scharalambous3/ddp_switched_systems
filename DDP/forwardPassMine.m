 function [uhat] = forwardPass(A, B, kArr, KArr, dyn_fun, xTraj, uTraj, tRange, alpha)
 % For a very small subtlety this doesnt work.There must be some detail I
 % am missing. Proofs match (see handwritten) with the other one but this
 % doesnt work.
 
 
 
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
global dt
uhat = zeros(size(uTraj,1),length(tRange));
x0 = xTraj(:,1);
xhat=zeros(size(xTraj,1),length(tRange));
xhat(:,1)=x0;

% A = zeros(size(xTraj,1), size(xTraj,1), length(tRange));
% B = zeros(size(xTraj,1), size(uTraj,1), length(tRange));
% 

%deltax = [0,0]';
for i = 1:length(tRange)-1
    
    x_i= xTraj(:,i);
    % feedforward modification term
    k_i = kArr(:,i);
    % feedback gain matrix
    K_i = KArr(:,:,i);
    % 
    deltax = (xhat(:,i)-x_i);
    ff_term = k_i;
    fb_term = K_i * deltax;
    deltau_star = ff_term + fb_term;
    uhat(:,i) = uTraj(:,i) + alpha * deltau_star;
    %[~, xRollout] = dyn_rollout(dyn_fun,xhat(:,i),uhat(:,i), [tRange(i),tRange(i+1)]);
    %xhat(:,i+1) = xRollout(:,end);
    %deltax = f_x * xhat(:,i) + f_u * uhat(:,i);
%     dynMatrices = dyn(tRange(i), xTraj(:,i), uTraj(:,i));
%     A(:,:,i) = dynMatrices.fx;
%     B(:,:,i) = dynMatrices.fu;
    f_x= eye(size(xTraj,1),size(xTraj,1)) + A(:,:,i)*dt;
    f_u= B(:,:,i)*dt;
    
    %xhat(:,i+1) = xhat(:,i) + f_x * deltax + f_u * deltau_star;
    % THis is what originally worked below
    %xhat(:,i+1) = xhat(:,i) + dyn_fun(0, xhat(:,i), uhat(:,i)) * dt;
    xhat(:,i+1) = f_x*xhat(:,i) +f_u*uhat(:,i);
    
    
    
    
end
end

