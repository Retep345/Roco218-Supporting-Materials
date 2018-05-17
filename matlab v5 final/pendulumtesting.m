clear all

close all

clc



l = 0.64;
m = 0.314;
u = 0.05;
I = 0; %for now assume 0
g = 9.81;

A3 = (m*g*l)/(I+(m*l*l));
A4 = -(u)/(I+(m*l*l));

B1 = (m*l)/(I+(m*l*l));
B2 = (-u*m*l)/((I+(m*l*l))^2);


A = [0,1;A3,A4];
B = [B1;B2];
C = [1,0];
D = 0;

% time steps of 100ms for integration
timeStep = 0.01;
% total time for simulation
totalTime = 1;
% build timepoint vector
tspan = 0:timeStep:totalTime;


eig(A);

Px = 20* [-1 -1.2]

k = place(A,B,Px)

eig(A-(B*k))

L = place(A, C', Px)



y0 = [0;3];

%[t,y] = ode45(@(t,y)SSSimulate(y,A,B,k),tspan,y0)

%[y, t, xout] = SimulateArduionSFC(A, B, C, D, K, t, x0)
[y, t, xout] = SimulateArduionSFC(A, B, C, D, k, L, tspan, y0)



range=1;
len = length(tspan);
kickFlag = zeros(1,len);

% get variables
x = 0*y(1, :);    % cart positon
th = y(1, :);   % pendulum angle



figure
AnimatePendulumCart(th+pi, x, l/2, tspan, range, kickFlag, 'pendulumTest');

figure
hold on;
plot (tspan, th, 'r-');
title("Time-Waveform Graph");
xlabel("Time");
ylabel("Angle");


%SYS = ss(A,B,C,D)

%step(SYS)

