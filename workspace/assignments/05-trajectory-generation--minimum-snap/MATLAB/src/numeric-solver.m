clc; clear; close all;

% press Enter to terminate waypoint selection:
path = ginput() * 100.0;

t_order       = 4;              % minimum jerk / minimum snap optimization
n_order       = 2*t_order - 1;  % order of polynomial trajectory
K             = size(path,1)-1; % segment number
N             = (n_order+1);    % coef number of perseg

% time allocation:
ts = zeros(K, 1);
% calculate time distribution in proportion to distance between 2 points
% dist     = zeros(K, 1);
% dist_sum = 0;
% T        = 25;
% t_sum    = 0;
% 
% for i = 1:K
%     dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
%     dist_sum = dist_sum+dist(i);
% end
% for i = 1:K-1
%     ts(i) = dist(i)/dist_sum*T;
%     t_sum = t_sum+ts(i);
% end
% ts(K) = T - t_sum;

% or you can simply set all time distribution as 1
for i = 1:K
    ts(i) = 1.0;
end

% plan the minimum snap trajectory
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, K, t_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, K, t_order);


% display the trajectory
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:K-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment
    %#####################################################
    % 1. get segment index:
    segment_index = (i*N + 1):((i+1)*N);
    % 2. extract segment coeffs:
    Pxi = flipud(poly_coef_x(segment_index));
    Pyi = flipud(poly_coef_y(segment_index));
    % 3. calculate planned waypoints:
    for t = 0:tstep:ts(i+1)
        X_n(k) = polyval(Pxi, t / ts(i + 1));
        Y_n(k) = polyval(Pyi, t / ts(i + 1));
        k = k + 1;
    end
end

% visualize the planned trajectory:
plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
hold on; grid on;
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));

% minimum snap trajectory generator:
function poly_coef = MinimumSnapQPSolver(waypoints, ts, K, t_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    %#####################################################
    Q = getQ(K, t_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq 
    %#####################################################
    [Aeq, beq] = getAbeq(K, t_order, waypoints, ts, start_cond, end_cond);
    %#####################################################
    % STEP 3: solve the problem with QP 
    %#####################################################
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end