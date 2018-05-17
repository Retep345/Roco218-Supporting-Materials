function xDot = SSSimulate(X, A, B, u)
% state space model
%X is state
%A,B are state space matrices
% u is their control input
%xdot is the returned 1st time derivative of state
xDot = (A * X) + (B * u);