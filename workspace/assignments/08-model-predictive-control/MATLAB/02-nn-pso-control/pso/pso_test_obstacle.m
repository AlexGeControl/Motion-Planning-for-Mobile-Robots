clear all;
close all;
clc;
%% Load all related modules
addpath ../
addpath ../NN_smooth_1
addpath ../DirectMethods/
global M hMap Map;
load ../NN_smooth_1/NN.mat
load Map_office_1;
%--------------------------------------------------------------------------
% Note: On this map, we have the following transfer relationship
% pixel = 3*(pos+10)+1;
% where pixel is the pixel position on the Map_office_1 cost image
% and pos is the real position of the vehicle
%--------------------------------------------------------------------------
%% Initialize the vehicle state
x=60;
y=18;
theta = 2.5129;
omega = 0;
v_c = 0;
dt = 0.02;

%% Create the logging list
pos = [];
theta_target = 0;
v_target = 0;
tgt_list=[];
a_list=[];

%% Simulation starts here
for n=1:300
    % Use the PSO algorithm to selet the best velocity and angular velocity
    % target
    global_best = pso_select(theta,omega,v_c,[x y]',theta_target,v_target);
    theta_target = global_best(1);
    v_target = global_best(2);
    
    % Prepare the translational trajectory
    a_star = sign(v_target-v_c)*2;
    if v_target~=v_c
        t_acc = (v_target-v_c)/a_star;
    else
        t_acc = 0;
    end
    
    % Prepare the angular trajectory
    ini.p = 0;
    ini.v = theta;
    ini.a = omega;
    [success,thetaL,wL]=second_order_trajectory(omega,theta_target-theta);
    thetaL = thetaL+theta;
    
    % Forward simulation of the vehicle
    for t=dt:dt:0.1
        % First get the angular part
        idx = round(t/dt+1);
        if idx > length(thetaL)
            idx = length(thetaL);
        end
        theta = thetaL(idx);
        omega = wL(idx);
        
        % Second get the translational part
        if t<=t_acc
            v_c = v_c+a_star*dt;
        else
            v_c= v_target;
        end
        
        % Move the vehicle according to a simple model
        x = x+dt*v_c*cos(theta);
        y = y+dt*v_c*sin(theta);
        
        % Log the trajectory
        pos =[pos;x y];
        a_list=[a_list;v_c theta];
    end
    n
end
%% Plot the result
close all;
imagesc(Map);hold on;
P=zeros(length(pos),2);

% Calculate the trajectory position on the image
for i=1:length(pos)
    P(i,1) = 3*(pos(i,2)+10)+1;
    P(i,2) = 3*(pos(i,1)+10)+1;
end

h=plot(P(:,1),P(:,2),'--','linewidth',2);
hold on;
for i=1:40:length(pos)
    sanjiao(P(i,1),P(i,2),-a_list(i,2)+pi/2,h.Color);
end
grid on;axis equal;

figure;
plot(pos);

figure;
plot(a_list);
