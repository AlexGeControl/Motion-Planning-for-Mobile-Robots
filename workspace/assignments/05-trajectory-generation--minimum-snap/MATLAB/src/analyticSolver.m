clc;clear;close all;

% press Enter to terminate waypoint selection:
path = ginput() * 100.0;

t_order       = 4;              % minimum jerk / minimum snap optimization
n_order       = 2*t_order - 1;  % order of polynomial trajectory
K             = size(path,1)-1; % segment number
N             = (n_order+1);    % coef number of perseg

% time allocation:
ts = zeros(K, 1);
% calculate time distribution based on distance between 2 points
% dist = zeros(K, 1);
% dist_sum = 0;
% T = 25;
% 
% t_sum = 0;
% for i = 1:K
%     dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
%     dist_sum = dist_sum + dist(i);
% end
% for i = 1:(K-1)
%     ts(i) = dist(i) / dist_sum * T;
%     t_sum = t_sum + ts(i);
% end
% ts(K) = T - t_sum;
% or you can simply average the time
for i = 1:K
    ts(i) = 1.0;
end

% plan the minimum snap trajectory
poly_coef_x = MinimumSnapCloseformSolver(path(:, 1), ts, K, t_order);
poly_coef_y = MinimumSnapCloseformSolver(path(:, 2), ts, K, t_order);

% display the trajectory
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:(K-1)
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
plot(X_n, Y_n ,'Color',[0 1.0 0],'LineWidth',2);
hold on; grid on;
scatter(path(1:size(path,1),1),path(1:size(path,1),2));

% minimum snap trajectory generator:
function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, K, t_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond =   [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    %#####################################################
    Q = getQ(K, t_order, ts);
    %#####################################################
    % STEP 2: compute M
    %#####################################################
    M = getM(K, t_order, ts);
    
    %#####################################################
    % STEP 3: compute C
    %#####################################################
    C = getC(K, t_order);
    
    %#####################################################
    % STEP 4: solve unconstrained optimization
    %#####################################################
    T = M \ C;
    Q = T' * Q * T;
    
    % 4.1 set boundary waypoints (start & end) states
    D = 2*t_order + (K - 1);
    N = (K+1)*t_order;
    
    d_fixed = zeros(D, 1);
    for c = 1:t_order
        d_fixed((c-1)*2 + 1) = start_cond(c);
        d_fixed((c-1)*2 + 2) = end_cond(c);
    end
    % 4.2 set intermediate waypoint positions:
    for k = 2:K
        d_fixed(2*t_order + k - 1) = waypoints(k);
    end
    
    % 4.3 solve least squared:
    Q_ff = Q((D + 1):N, (D + 1):N);
    Q_fd = Q((D + 1):N, 1:D);
    d_free = -inv(Q_ff)*Q_fd*d_fixed;

    %#####################################################
    % STEP 5: restore poly coef
    %#####################################################
    poly_coef = T * [d_fixed; d_free];
end