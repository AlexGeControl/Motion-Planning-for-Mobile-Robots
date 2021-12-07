% reset session:
clear all; close all; clc;

% ###############################################
% problem specification
% ###############################################
T     = 40;
dt    = 0.2;                 % time interval
K     = 20;                  % prediction horizon

w_pos = 100.0;               % position tracking weight
w_vel =   1.0;               % velocity trackng weight
w_acc =   1.0;               % acceleration tracking weight
w_jer =   1.0;               % control strength regulation

vel_limits = [
    [-6, +6];
    [-6, +6];
    [-1, +6]; 
]';                          % velocity limits
acc_limits = [
    [-3, +3];
    [-3, +3];
    [-1, +3];
]';                          % acceleration limits
jer_limits = [
    [-3, +3];
    [-3, +3];
    [-2, +2];
]';                          % jerk limits

% ###############################################
% solve problem
% ###############################################
% output trajectory:
N = (T / dt);
state_index = 1;

timestamps = zeros(N, 1);

P_act = zeros(N, 3);
V_act = zeros(N, 3);
A_act = zeros(N, 3);

P_req = zeros(N, 3);
V_req = zeros(N, 3);
A_req = zeros(N, 3);

% define init state:
p_0 = [0 8 20];
v_0 = [0 0  0];
a_0 = [0 0  0];

for t = 0.2:dt:40
    % ###############################################
    % identify target trajectory ahead
    % ###############################################
    pt = zeros(K, 3);
    vt = zeros(K, 3);
    at = zeros(K, 3);
    for i = 1:K
        tref = t + i*dt;
        r=0.25*tref;
        pt(i,1) =  r*sin(0.2*tref);
        vt(i,1) =  r*cos(0.2*tref);
        at(i,1) = -r*sin(0.2*tref);
        
        pt(i,2) =  r*cos(0.2*tref);
        vt(i,2) = -r*sin(0.2*tref);
        at(i,2) = -r*cos(0.2*tref);
        
        pt(i,3) = 20 - 0.5*tref;
        vt(i,3) = -0.5;
        at(i,3) = 0;
    end

    % ###############################################
    % solve MPC
    % ###############################################
    J = zeros(K, 3);
    for i = 1:3 
        % get prediction matrix:
        [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0(i),v_0(i),a_0(i));

        % build objective function:
        H = w_pos*(Tp'*Tp) + w_vel*(Tv'*Tv) + w_acc*(Ta'*Ta) + w_jer*eye(K);
        f = w_pos*(Bp - pt(:, i))'*Tp + w_vel*(Bv - vt(:, i))'*Tv + w_acc*(Ba - at(:, i))'*Ta;

        % build constraint matrix:
        A_ieq = [
                 Tv;
                -Tv;
                 Ta;
                -Ta;
             eye(K);
            -eye(K);
        ];
        b_ieq = [
            -Bv + vel_limits(2, i);
             Bv - vel_limits(1, i); 
            -Ba + acc_limits(2, i);
             Ba - acc_limits(1, i); 
             jer_limits(2, i) * ones(K, 1);
            -jer_limits(1, i) * ones(K, 1);
        ];

        % solve the problem:
        J(:, i) = quadprog(H,f,A_ieq,b_ieq);
    end

    % ###############################################
    % apply optimal control
    % ###############################################
    j = J(1, :);
    for i = 1:3
       [p_0(i),v_0(i),a_0(i)] = forward(p_0(i),v_0(i),a_0(i),j(i),dt);
    end

    % log the states:
    timestamps(state_index) = state_index;

    P_act(state_index, :) = p_0;
    V_act(state_index, :) = v_0;
    A_act(state_index, :) = a_0;

    P_req(state_index, :) = pt(1, :);
    V_req(state_index, :) = vt(1, :);
    A_req(state_index, :) = at(1, :);

    state_index = state_index + 1;
end

% ###############################################
% visualize tracking results
% ###############################################
% visualize trajectory:
figure;
plot3(P_req(:,1),P_req(:,2),P_req(:,3), P_act(:,1),P_act(:,2),P_act(:,3), '--');
axis equal;
grid on;
legend('Required', 'Actual');

% visualize tracking error:
figure;
err_pos = vecnorm(P_req - P_act, 2, 2);
err_vel = vecnorm(V_req - V_act, 2, 2);
err_acc = vecnorm(A_req - A_act, 2, 2);
plot(timestamps, [err_pos, err_vel, err_acc]);
grid on;
xlabel('t(s)');
legend('Pos. Err.','Vel. Err.','Acc. Err.');