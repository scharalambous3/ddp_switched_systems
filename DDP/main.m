 clc
clear
close all

global dt;

%Timestep
dt = 0.001;
%Vector of time horizon
tRange=[0:dt:1];

% %Dynamics of double integrator
% 1: double integrator, 2: cart_pendulum
typeProb = 2;

switch typeProb
     case 1
        dyn=@double_integrator_dynMatrices;
        dyn_fun = @double_integrator_dyn;
        % State size
        n=2;
        % Initial and desired state
        x0=[0,0]';
        xDes = [1,0]';
        % Cost weighting matrices
        Q = eye(n);
        R = 1;
        Qf = 1000*eye(n);
     case 2
        dyn= @cart_pend_dynMatrices;
        dyn_fun = @cart_pend_dyn;
        % State size
        n=4;
        % Initial and desired state
        x0=[0,0,0.1,0]';
        xDes = [-1,0,pi,0]';
        
        
        % Cost weighting matrices
        Q = 1*diag([5000,100,15000,100]);
        R = 15*eye(1,1);
        Qf = diag([5000,100,15000,100]);
end


numIterations=400;
cost=[]
runningCost=[]
finalCost=[]
uTraj = zeros(1,length(tRange));
%MPC
for i = 1: numIterations



    % Reference trajectories
    xDesTraj = repmat(xDes,1, length(tRange));
    uDesTraj = zeros(1,length(tRange));

    
    % Rollout dynamics
    [t, xTraj] =  dyn_rollout_discrete(dyn_fun, x0, uTraj, tRange);

    
    
    
    % IMPLEMENT LINE SEARCH STRATEGY
    
    %Get feed-forward and feedback gain matrix
    finalCostStruct = getFinalCost(Qf, xTraj, xDesTraj);
    [kArr, KArr, A, B] = backwardPass(dyn, finalCostStruct, Q, R, tRange, xTraj, uTraj, xDesTraj, uDesTraj);
    uTrajSoln = forwardPass(kArr, KArr, dyn, dyn_fun, xTraj, uTraj, tRange, 0.15);
    [tSoln, xTrajSoln] =  dyn_rollout_discrete(dyn_fun, x0, uTrajSoln, tRange);
    runningCostNew = getTotalRunningCost(Q,R, tRange, xTrajSoln, uTrajSoln, xDesTraj, uDesTraj);
    finalCostStructNew = getFinalCost(Qf, xTrajSoln, xDesTraj);
    finalCostNew = finalCostStructNew.value;
    runningCost = [runningCost,runningCostNew];
    finalCost = [finalCost, finalCostNew];
    % getTotalRunningCost(Q,R, tRange, xTraj, uTraj, xDesTraj, uDesTraj)
    uTraj = uTrajSoln;
    
end
cost = runningCost + finalCost;
%%


figure(1)
plot(1:numIterations,cost)

figure(3)
hax = axes

switch typeProb
     case 1
         drawdoubleintegrator(hax, xTraj,tSoln,xDesTraj(1,1))
         figure(2)
         subplot(1,2,1)
         plot(tRange, uTrajSoln)
         subplot(1,2,2)
         plot(tRange, xTrajSoln(1,1:end-1))
     case 2
         for k=1:length(tRange)
             m = 1;
             M = 5;
             L = 2;
             drawcartpend(hax,xTrajSoln(:,k),m,M,L);
         end

 end