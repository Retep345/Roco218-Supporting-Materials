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


A = [0,1;-A3,A4];
B = [B1;B2];
C = [1,0];
D = 0;

% time steps of 100ms for integration
timeStep = 0.1;
% total time for simulation
totalTime = 10;
% build timepoint vector
tspan = 0:timeStep:totalTime;

y0 = [pi;0];

[t,y] = ode45(@(t,y)SSSimulate(y,A,B,0),tspan,y0);

range=1;
len = length(tspan);
kickFlag = zeros(1,len);

% get variables
x = 0*y(:, 1);    % cart positon
th = y(:, 1);   % pendulum angle


figure
AnimatePendulumCart(th, x, l/2, tspan, range, kickFlag, 'pendulumTest');

figure
hold on;
plot (tspan, th, 'r-');
title("Time-Waveform Graph");
xlabel("Time");
ylabel("Angle");

%eig(A)

%SYS = ss(A,B,C,D)

%step(SYS)

