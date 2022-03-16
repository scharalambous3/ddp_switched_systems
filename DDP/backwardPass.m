function [kArr, KArr, A, B] = backwardPass(dyn, finalCost, Q, R, tRange, xTraj, uTraj, xDesTraj, uDesTraj)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%Preallocate value function array
V_xx=zeros(size(xTraj,1), size(xTraj,1), length(tRange));
V_x=zeros(size(xTraj,1),length(tRange));
V=zeros(length(tRange));

global dt
%Initialize quadratic model of value function with the terminal cost and its derivatives
V(length(tRange)) = finalCost.value;
V_x(:,length(tRange)) = finalCost.dx;
V_xx(:,:,length(tRange)) = finalCost.dxx;

kArr = zeros(size(uTraj,1), length(tRange));
KArr = zeros(size(uTraj,1),size(xTraj,1), length(tRange));
Q_xx = zeros(size(xTraj,1),size(xTraj,1));
Q_ux = zeros(size(uTraj,1),size(xTraj,1));
Q_uu = zeros(size(uTraj,1),size(uTraj,1));

A = zeros(size(xTraj,1), size(xTraj,1), length(tRange));
B = zeros(size(xTraj,1), size(uTraj,1), length(tRange));


for i = (length(tRange)-1):-1:1
    x = xTraj(:,i);
    u = uTraj(:,i);
    cost = getCost(Q, R, tRange(i), tRange, x, u, xDesTraj, uDesTraj);
    dynMatrices = dyn(tRange(i), x, u, tRange);
    A(:,:,i) = dynMatrices.fx;
    B(:,:,i) = dynMatrices.fu;
    
    
    f_x= eye(size(xTraj,1),size(xTraj,1)) + A(:,:,i)*dt;
    f_u= B(:,:,i)*dt;
    
    %TODO: fix quadratic model of dynamics for tensor product
    f_xx=dynMatrices.fxx*dt;
    f_uu=dynMatrices.fuu*dt;
    f_ux=dynMatrices.fux*dt;
    
    Q_x= cost.dx + f_x'*V_x(:,i+1);
    Q_u= cost.du + f_u'*V_x(:,i+1);
    for r = 1:size(Q_xx,1)
        for c = 1:size(Q_xx,2)
            xxTensor = reshape(f_xx(r,c,:),1,size(V_x(:,i+1),1));
            xx_tensorProd(r,c) = xxTensor*V_x(:,i+1);
        end
    end
    for r = 1:size(Q_ux,1)
        for c = 1:size(Q_ux,2)   
            uxTensor = reshape(f_ux(r,c,:),1,size(V_x(:,i+1),1));
            ux_tensorProd(r,c) = uxTensor*V_x(:,i+1);
        end
    end
    for r = 1:size(Q_uu,1)
        for c = 1:size(Q_uu,2)
            uuTensor = reshape(f_uu(r,c,:),1,size(V_x(:,i+1),1));
            uu_tensorProd(r,c) = uuTensor*V_x(:,i+1);
        end
    end
    Q_xx= cost.dxx + f_x'*V_xx(:,:,i+1)*f_x;% + xx_tensorProd;
    Q_ux= cost.dux + f_u'*V_xx(:,:,i+1)*f_x;% + ux_tensorProd;
    Q_uu= cost.duu + f_u'*V_xx(:,:,i+1)*f_u;% + uu_tensorProd;
    
    k = - inv(Q_uu)* Q_u;
    K = - inv(Q_uu)* Q_ux;
    %V_x(:,i) = Q_x - K'*Q_uu*k;  
    %V_xx(:,:,i) = Q_xx - K'*Q_uu*K;
    
    V_x(:,i) = Q_x + K'*Q_uu*k +K'*Q_u+Q_ux'*k;  
    V_xx(:,:,i) = Q_xx + K'*Q_uu*K +K'*Q_ux+Q_ux'*K;
    
    %V_x(:,i) = Q_x - Q_u *     inv(Q_uu) * Q_ux;
    %V_xx(:,:,i) = Q_xx - Q_xu * inv(Q_uu) * Q_ux;
    
    
    
    kArr(:,i) = k;
    KArr(:,:,i) = K;
    
end

end   