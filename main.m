%% Clear variables, cmd window and plots
clc
clear
close all

%% Parameters and problem type
global dt;
global fxFunc fuFunc;

% Timestep
dt = 0.01;
% Time trajectory
tRange=[0:dt:10];

%Specify problem type:
% 1 for double integrator; 2 for cart pendulum
typeProb = 2;

%%
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
        M = 1;
        m = M/100;
        L = 1/4;
        g = -9.81;
        syms xVar xdot theta thetadot uVar;
        dxdt1 = xdot;
        dxdt2 = (uVar + m*sin(theta)*(L*thetadot^2 - g * cos(theta)))/(M+m*sin(theta)^2);
        dxdt3 = thetadot;
        dxdt4 = (-uVar*cos(theta) - m*L*thetadot^2*sin(theta)*cos(theta)+(M+m)*g*sin(theta))/(L*(M+m*sin(theta)^2));

        
        fxFunc = matlabFunction(jacobian([dxdt1; dxdt2; dxdt3; dxdt4], [xVar, xdot, theta, thetadot]));
        fuFunc = matlabFunction(jacobian([dxdt1; dxdt2; dxdt3; dxdt4], [uVar]));
        dyn= @cart_pend_dynMatrices;
        dyn_fun = @cart_pend_dyn;
        % State size
        n=4;
        % Initial and desired state
        x0=[0,0,0,0]';
        xDes = [-1,0,pi,0]';
        
        
        % Cost weighting matrices
        Q = 0*diag([5000,100,15000,100]);
        R = 15*eye(1,1);
        Qf = diag([5000,100,15000,100]);
end

%Maximum number of iterations
numIterations=100;
%Minimum number of iterations (must be >= 10)
minIters=20;
cost=[];
runningCost=[];
finalCost=[];
uTraj = zeros(1,length(tRange));
%MPC
for i = 1: numIterations

    

    % Reference trajectories
    xDesTraj = repmat(xDes,1, length(tRange));
    uDesTraj = zeros(1,length(tRange));

    
    % Rollout dynamics
    [t, xTraj] =  dyn_rollout_discrete(dyn_fun, x0, uTraj, tRange);
     
    %Get feed-forward and feedback gain matrix
    finalCostStruct = getFinalCost(Qf, xTraj, xDesTraj);
    [kArr, KArr, A, B] = backwardPass(dyn, finalCostStruct, Q, R, tRange, xTraj, uTraj, xDesTraj, uDesTraj);
    % TODO: Implement line search strategy
    backtrackingLineSearchFlag =  true;
    learningRate=1;
    rho = 0.8;
    costLearningRate=[];
    learningRateVec=[];
    while backtrackingLineSearchFlag
        uTrajSoln = forwardPass(A,B,kArr, KArr, dyn_fun, xTraj, uTraj, tRange, learningRate);

        [tSoln, xTrajSoln] =  dyn_rollout_discrete(dyn_fun, x0, uTrajSoln, tRange);
        runningCostNew = getTotalRunningCost(Q,R, length(tRange), xTrajSoln, uTrajSoln, xDesTraj, uDesTraj);
        finalCostStructNew = getFinalCost(Qf, xTrajSoln, xDesTraj);
        finalCostNew = finalCostStructNew.value;
        
        costLearningRate=[costLearningRate, (runningCostNew+finalCostNew)];
        learningRateVec = [learningRateVec, learningRate];
        if i == 1
            runningCost = [runningCost,runningCostNew];
            finalCost = [finalCost, finalCostNew];
            backtrackingLineSearchFlag = false;
        elseif (((runningCostNew + finalCostNew) < (runningCost(end) + finalCost(end)))) || learningRate < 0.1            %Get running and terminal cost of current iteration
            runningCost = [runningCost,runningCostNew];
%             getTotalRunningCost(Q,R, tRange, xTraj, uTraj, xDesTraj, uDesTraj)
            finalCost = [finalCost, finalCostNew];
            backtrackingLineSearchFlag = false;
        else
            learningRate= rho * learningRate;
        end
    end
%     hold on
%     plot(learningRateVec, costLearningRate)
%     yline(runningCost(end) + finalCost(end))
%     hold off
    uTraj = uTrajSoln;
    if i > minIters
        cost = runningCost + finalCost;
        if std(cost(end-10:end))/cost(end)<0.1
            break
        end
    end
end
%%


figure(1)
plot(1:i,cost)
xlabel('Iteration')
ylabel('Total cost')

figure(2)
subplot(1,2,1)
plot(tRange, uTrajSoln)
xlabel('Time')
ylabel('Optimal input')
figure(2)
subplot(1,2,2)
plot(tRange, xTrajSoln)
switch typeProb
     case 1
         legend("x-Position","x-Velocity")
    case 2
        legend("x-Position","x-Velocity","Theta","Theta dot")
end
xlabel('Time')
ylabel('Optimal state')

figure(3)
hax = axes

%gif('A.gif','DelayTime',1/5)
desFrames=5;
firstFrame=true;
jumpStep = floor(length(tRange)/desFrames);
switch typeProb
     case 1
         for k=1:length(tRange)
             if mod(k, desFrames) ~=0
                 continue
             end
             drawdoubleintegrator(hax, xTraj(1,k),xDesTraj(1,k));
             if firstFrame
                firstFrame=false;
                %pause
             end
         end
     case 2
         for k=1:length(tRange)
             if mod(k, desFrames) ~=0
                 continue
             end
             m = 1;
             M = 5;
             L = 2;
             drawcartpend(hax,xTrajSoln(:,k),m,M,L);
             if firstFrame
                firstFrame=false;
                %pause
             end
             %gif
         end

 end