clear all;
close all;
clc;

p_0 = 10;
v_0 = -3;
a_0 = 0;
K=20;
dt=0.2;

log=[0 p_0 v_0 a_0];
w1 = 10;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 1e4;
for t=0.2:0.2:20
    %% Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);
    
    %% Construct the optimization problem
    H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
    F = [w1*Bp'*Tp+w2*Bv'*Tv+w3*Ba'*Ta zeros(1,K)];
    
    A = [Tv zeros(K);-Tv -eye(K);Ta zeros(K); -Ta zeros(K); zeros(size(Ta)) -eye(K)];
    b = [ones(20,1)-Bv;ones(20,1)+Bv;ones(20,1)-Ba;ones(20,1)+Ba; zeros(K,1)];
    %% Solve the optimization problem
    J = quadprog(H,F,A,b);
    
    %% Apply the control
    j = J(1);
    p_0 = p_0 + v_0*dt + 0.5*a_0*dt^2 + 1/6*j*dt^3;
    v_0 = v_0 + a_0*dt + 0.5*j*dt^2;
    a_0 = a_0 + j*dt; 
    
    %% Log the states
    log = [log; t p_0 v_0 a_0];
end

%% Plot the result
plot(log(:,1),log(:,2:4));
grid on;
xlabel('t(s)');
legend('p','v','a');
