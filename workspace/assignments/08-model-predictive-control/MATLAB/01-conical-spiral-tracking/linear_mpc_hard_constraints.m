clear all;
close all;
clc;

p_0 = 10;
v_0 = 0;
a_0 = 0;
K=20;
dt=0.2;

log=[0 p_0 v_0 a_0];
w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
for t=0.2:0.2:20
    %% Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);
    
    %% Construct the optimization problem
    H = w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta);
    F = w1*Bp'*Tp+w2*Bv'*Tv+w3*Ba'*Ta;
    
    A = [Tv;-Tv;Ta;-Ta];
    b = [ones(20,1)-Bv;ones(20,1)+Bv;ones(20,1)-Ba;ones(20,1)+Ba];
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
