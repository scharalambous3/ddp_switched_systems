# Differential Dynamic Programming
Implementation of DDP/iLQR for linear, nonlinear and switched systems



Double integrator example
<p style="text-align:center;"><img src="https://github.com/scharalambous3/ddp_switched_systems/blob/main/doubleint.gif" alt="Logo"></p>

Cart pendulum example
<p style="text-align:center;"><img src="https://github.com/scharalambous3/ddp_switched_systems/blob/main/cartpend.gif" alt="Logo"></p>

**Functions**:
- backwardPass: Computes quadratic model of value function using backwards recursion to output feedback and feedforward terms, and dynamic matrices of linear dynamics along reference state and input trajectories
- forwardPass: Updates input sequence using the feedforward and feedback arrays computed from backward pass
- getCost: Returns quadratic approximation of stage cost at a specific stage for specific state and input values
- getFinalCost: Returns quadratic approximation of terminal cost for specific state value
- getTotalRunningCost: Returns stage cost integrated over given state and input trajectories
- dyn_rollout: Integrates trajectory forward from a given initial condition and input sequence (continuous time)
- dyn_rollout_discrete: Integrates trajectory forward from a given initial condition and input sequence (discrete time)
- interpolateTraj: Interpolates given trajectory at a specific time linearly
- vis/: visualization functions

TODO:
- [ ] Backtracking-Armijo Line Search
- [ ] Add noise/ LQG
- [ ] Extend to switched systems
- [ ] Add Switching Time Optimization
