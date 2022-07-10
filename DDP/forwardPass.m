 function [uhat] = forwardPass(A, B, kArr, KArr, dyn_fun, xTraj, uTraj, tRange, alpha)
%forwardPass Update input sequence using arrays of feedforward and feedback
%terms from backward pass
global dt
uhat = zeros(size(uTraj,1),length(tRange));
x0 = xTraj(:,1);
xDim = length(x0);
xhat=zeros(size(xTraj,1),length(tRange));
xhat(:,1)=x0;

deltax = zeros(size(xTraj,1),1);

for i = 1:length(tRange)-1
    %Obtain linear discrete dynamics about given state and input
    %trajectories
    fx = eye(xDim, xDim) + A(:,:,i)  * dt;
    fu= B(:,:,i) * dt;
    
    %Compute input update using feedforward and feedback terms
    deltau = kArr(:,i) + KArr(:,:,i) * deltax;
    deltax = fx * deltax + fu * deltau; 
    %Compute updated input term
    uhat(:,i) = uTraj(:,i) + alpha * deltau;
end
end

