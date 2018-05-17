function [y, t, xout] = SimulateArduionSFC(A, B, C, D, K, t, x0)

%A,B,C,D are matrices, k is state feedback gain
%t = time of each sample, x0 = initial state
%returns y, each time t and state are updated

%signal length
len = length(t);

%initial output
y = zeros(1,len);
xout = zeros(2,len);

%initial state
xout(:, 1) = x0;
x = x0;

%command
u(1)= C(1)*x(1) + C(2)*x(2);

%theta and thetadot
y(1)= C(1)*x(1) + C(2)*x(2) +D(1)*u(1);


for idx = 2:len
    
    %SF rule
    u(idx) = -K(1)*x(1) - K(2) * x(2);
    
    %duration
    h = t(idx) - t(idx-1);
    
    %state derivative
    xdot(1) = A(1,1)*x(1) + A(1,2)*x(2) + B(1)*u(idx);
    xdot(2) = A(2,1)*x(1) + A(2,2)*x(2) + B(2)*u(idx);
     
    %update state
    x(1) = x(1) + h * xdot(1);
    x(2) = x(2) + h * xdot(2);
    
    %record state
    xout(:, idx) = x;
    
    %calculate output from theta and thetadot only
    y(idx) = C(1)*x(1) + C(2)*x(2) +D(1)*u(idx);
    
end
    
    
    
    
    
    
    
