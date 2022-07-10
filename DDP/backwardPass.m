function [kArr, KArr, A, B] = backwardPass(dyn, finalCost, Q, R, tRange, xTraj, uTraj, xDesTraj, uDesTraj)
%backwardPass 
%Preallocate value function array
V_xx=zeros(size(xTraj,1), size(xTraj,1), length(tRange));
V_x=zeros(size(xTraj,1),length(tRange));
V=zeros(length(tRange));

%Horizon length
hor = length(tRange);

global dt
%Initialize quadratic model of value function with the terminal cost and
%its derivatives
V(hor) = finalCost.value;
V_x(:, hor) = finalCost.dx;
V_xx(:,:, hor) = finalCost.dxx;

kArr = zeros(size(uTraj,1), length(tRange)-1);
KArr = zeros(size(uTraj,1),size(xTraj,1), length(tRange)-1);
Q_xx = zeros(size(xTraj,1),size(xTraj,1));
Q_ux = zeros(size(uTraj,1),size(xTraj,1));
Q_uu = zeros(size(uTraj,1),size(uTraj,1));

%Preallocate array of dynamic matrices of linear dynamics along reference
%state and input trajectories
A = zeros(size(xTraj,1), size(xTraj,1), length(tRange)-1);
B = zeros(size(xTraj,1), size(uTraj,1), length(tRange)-1);


for i = (length(tRange)-1):-1:1
    % Obtain state and input at ith stage from given trajectories
    x = xTraj(:,i);
    u = uTraj(:,i);
    %Obtain quadratic approximation of cost about x and u
    cost = getCost(Q, R, i, x, u, xDesTraj, uDesTraj);
    %Obtain linear approximation of dynamics about x and u
    dynMatrices = dyn(tRange(i), x, u);
    A(:,:,i) = dynMatrices.fx;
    B(:,:,i) = dynMatrices.fu;
    
    %Transform to discrete time
    f_x= eye(size(xTraj,1),size(xTraj,1)) + A(:,:,i)*dt;
    f_u= B(:,:,i)*dt;
    
    %TODO: fix quadratic model of dynamics for tensor product
    f_xx=dynMatrices.fxx*dt;
    f_uu=dynMatrices.fuu*dt;
    f_ux=dynMatrices.fux*dt;
    
    %Linear terms of quadratic approximation of cost to go
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
    %Quadratic terms of quadratic approximation of cost to go
    Q_xx= cost.dxx + f_x'*V_xx(:,:,i+1)*f_x;% + xx_tensorProd;
    Q_ux= cost.dux + f_u'*V_xx(:,:,i+1)*f_x;% + ux_tensorProd;
    Q_uu= cost.duu + f_u'*V_xx(:,:,i+1)*f_u;% + uu_tensorProd;
    
    %Feedforward vector
    k = - inv(Q_uu)* Q_u;
    %Feedback matrix
    K = - inv(Q_uu)* Q_ux;

    %V_x(:,i) = Q_x - K'*Q_uu*k;  
    %V_xx(:,:,i) = Q_xx - K'*Q_uu*K;
    
    V_x(:,i) = Q_x + K'*Q_uu*k +K'*Q_u+Q_ux'*k;  
    V_xx(:,:,i) = Q_xx + K'*Q_uu*K +K'*Q_ux+Q_ux'*K;
    
    %V_x(:,i) = Q_x - Q_u *     inv(Q_uu) * Q_ux;
    %V_xx(:,:,i) = Q_xx - Q_xu * inv(Q_uu) * Q_ux;
    
    %Populate arrays
    kArr(:,i) = k;
    KArr(:,:,i) = K;
    
end

end   