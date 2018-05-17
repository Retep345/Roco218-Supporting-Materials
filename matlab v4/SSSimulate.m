function xDot = SSSimulate(X, A, B, k)
% state space model
%X is state
%A,B are state space matrices
% u is their control input
%xdot is the returned 1st time derivative of state
u=-k*X;


xDot = (A * X) + (B * u);