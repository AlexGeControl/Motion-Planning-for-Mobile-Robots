%==========================================================================
% set up session
%==========================================================================
% reset session:
clear all; close all; clc;

% load libraries:
addpath ../
addpath ../nn_scorer
addpath ../mpc/

global M hMap Map;

load ../nn_scorer/NN.mat
load map;

%--------------------------------------------------------------------------
% Note: On this map, we have the following transfer relationship
% pixel = 3*(pos+10)+1;
% where pixel is the pixel position on the map cost image
% and pos is the real position of the vehicle
%--------------------------------------------------------------------------

%==========================================================================
% problem configuration
%==========================================================================
% prediction time interval:
dt           = 0.02;

% max execution steps:
N            = 800;

% initial state of ego vehicle:
x_0          =     60;              % init se2 pose, position x
y_0          =     18;              % init se2 pose, position y
theta_0      = 2.5129;              % init se2 pose, heading

omega_0      = 0.0;                 % init angular velocity
v_0          = 0.0;                 % init longitudinal velocity

%==========================================================================
% solve problem 
%==========================================================================
% init ego vehicle state:
x     = x_0;
y     = y_0;
theta = theta_0;
omega = omega_0;
v     = v_0;

% init control: 
theta_target = 0.0;
v_target     = 0.0;

% init output trajectory:
pos = zeros(N, 2);
u   = zeros(N, 2);

% forward simulation:
for n = 1:N
    % use PSO to selet the best heading and longitudinal velocity:
    [theta_target, v_target] = pso_select(theta, [x y]', omega, v, theta_target,v_target);
    
    % prepare the translational trajectory
    a_star = sign(v_target-v)*2;
    if v_target~=v
        t_acc = (v_target-v)/a_star;
    else
        t_acc = 0;
    end
    
    % prepare the angular trajectory
    ini.p = 0;
    ini.v = theta;
    ini.a = omega;
    [success,thetaL,wL] = second_order_trajectory(omega, theta_target-theta);
    thetaL = thetaL+theta;
    
    % Forward simulation of the vehicle
    for t = dt:dt:0.1
        % set heading and angular velocity:
        idx = round(t/dt+1);
        if idx > length(thetaL)
            idx = length(thetaL);
        end
        theta = thetaL(idx);
        omega = wL(idx);
        
        % set longitudinal velocity:
        if t <= t_acc
            v = v + a_star*dt;
        else
            v = v_target;
        end
        
        % update ego vehicle state using kinematic model:
        x = x + v*cos(theta)*dt;
        y = y + v*sin(theta)*dt;
        
        % log trajectory
        pos(n, :) = [    x, y];
        u(n, :)   = [theta, v];
    end
end

%==========================================================================
% visualize the result 
%==========================================================================
% show map:
close all;
imagesc(Map);hold on;

%
% show trajectory:
%
% plot trajectory positions:
grid = zeros(length(pos),2);
for i = 1:length(pos)
    grid(i,1) = 3*(pos(i,2)+10)+1;
    grid(i,2) = 3*(pos(i,1)+10)+1;
end
h = plot(grid(:,1),grid(:,2),'--','linewidth',2);
hold on;

% plot trajectory headings:
for i=1:4:length(pos)
    sanjiao(grid(i,1), grid(i,2), -u(i,1)+pi/2, h.Color);
end
grid on; axis equal;

% show travelled positions:
figure;
plot(pos); grid on;
xlabel('t(s)');
legend('x', 'y');

% show control:
figure;
plot(u); grid on;
xlabel('t(s)');
legend('theta', 'v');
