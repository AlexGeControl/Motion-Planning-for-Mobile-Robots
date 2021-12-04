% ####################################################################################
% reset session
% ####################################################################################
clc;clear all;close all;

% ####################################################################################
% motion planning config
% ####################################################################################
t_order     = 4;              % minimum jerk / minimum snap optimization
n_order     = 2*t_order - 1;  % order of polynomial trajectory
N           = 2*t_order;      % num. poly coeffs
v_max       = 400;            % vel. limit
a_max       = 200;            % acc. limit

% ####################################################################################
% flight corridor specification
%    1. flight corridor consists of a series of bounding boxes
%    2. each bounding box is defined by its center and corresponding width and length
% ####################################################################################
bbox_center = [
     50,  50;
    100, 120;
    180, 150;
    250,  80;
    280,   0
];
bbox_width  = 100;
bbox_length = 100;

K = size(bbox_center, 1);

corridor = zeros(K, 4);
for k = 1:K
    corridor(k, :) = [bbox_center(k, 1), bbox_center(k, 2), bbox_width/2, bbox_length/2];
end

% ####################################################################################
% time allocation
% ####################################################################################
ts = zeros(K, 1);
for i = 1:K
    ts(i,1) = 1;
end

poly_coef_x = MinimumSnapCorridorBezierSolver(1, bbox_center(:, 1), corridor(:, 3), ts, K, t_order, v_max, a_max);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, bbox_center(:, 2), corridor(:, 4), ts, K, t_order, v_max, a_max);

% ####################################################################################
% display the flight corridor and planned trajectory
% ####################################################################################
% color config:
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];

% flight corridor:
plot(bbox_center(:,1), bbox_center(:,2), '*r'); hold on;
for k = 1:K
    plot_rect([corridor(k,1);corridor(k,2)], corridor(k, 3), corridor(k,4));hold on;
end

% planned trajectory:
x_pos = [];y_pos = [];
idx = 1;
for k = 1:K
    % get segment index:
    segment_index = ((k-1)*N + 1):(k*N);
    % extract segment coeffs:
    Pxk = flipud(poly_coef_x(segment_index));
    Pyk = flipud(poly_coef_y(segment_index));
    for t = 0:0.01:1
        x_pos(idx) = polyval(Pxk, t);
        y_pos(idx) = polyval(Pyk, t);

        idx = idx + 1;
    end
end

plot(x_pos, y_pos, 'Color', [0 1.0 0], 'LineWidth', 2);
hold on; grid on;

% ####################################################################################
% numeric solver implementation
% ####################################################################################
function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, K, t_order, v_max, a_max)
    % boundary conditions:
    start_cond    = zeros(t_order, 1);
    start_cond(1) = waypoints(1);

    end_cond      = zeros(t_order, 1);
    end_cond(1)   = waypoints(end);   
    
    % build objective matrix:
    [P,M] = getPM(K, t_order, ts);
    q = zeros(size(P,1),1);

    % build constraint matrix, equality:
    [Aeq, beq] = getAbeq(K, t_order, ts, start_cond, end_cond);
    
    % build constraint matrix, inequality:
    corridor_range = zeros(K, 2);
    for k = 1:K
        corridor_range(k, 1) = waypoints(k) - corridor(k);
        corridor_range(k, 2) = waypoints(k) + corridor(k);
    end
    [Aieq, bieq] = getAbieq(K, t_order, ts, corridor_range, v_max, a_max);
    
    % solve coeffs, bernstein polynomial:
    poly_coef = quadprog(P,q, Aieq, bieq, Aeq, beq);

    % map to monomial coeffs:
    poly_coef = M * poly_coef;
end

function plot_rect(center, x_r, y_r)
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];    
    plot(a(1,:),a(2,:),'b');
end