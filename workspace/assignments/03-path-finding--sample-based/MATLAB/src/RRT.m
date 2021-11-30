%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化

clc;
clear all; close all;   % reset session

x_I=1; y_I=1;           % set start state
x_G=700; y_G=700;       % set goal state
Thr=50;                 % threshold for goal check
Delta= 30;              % config param for new child expansion
%% 建树初始化

T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % 从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     %
%% 开始构建树，作业部分

figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);%地图x轴长度
yL=size(Imp,1);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count=1;
bFind = false;

% STEP 0: init random number generator to make the results reproducible
seed = 42;
rng(seed, "twister");

x_goal = [x_G; y_G];

for iter = 1:3000
    % STEP 1: 在地图中随机采样一个点x_rand
    x_rand= [
        randi([1, xL], 1); 
        randi([1, yL], 1)
    ];
    
    % STEP 2: 遍历树，从树中找到最近邻近点x_near
    % a. calculate x_rand distance to every x_candidate:
    x_candidates = zeros(2, count);
    for i = 1:count
        x_candidates(:, i) = [
            T.v(i).x;
            T.v(i).y
        ];
    end
    distancesSquared = sum((x_candidates - x_rand).^2, 1);
    [~, indNear] = min(distancesSquared);
    % b. done:
    x_near=[
        T.v(indNear).x;
        T.v(indNear).y
    ];
    
    % STEP 3: 扩展得到x_new节点
    % a. get direction:
    direction = (x_rand - x_near);
    direction = direction / norm(direction);
    % b. move along direction for Delta
    x_new = x_near + Delta * direction;
    
    %检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
        continue;
    end
    count = count+1;
    
    % STEP 4: 将x_new插入树T 
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);
    T.v(count).yPrev = x_near(2);
    T.v(count).dist = Delta;
    T.v(count).indPrev = indNear;
    
    % STEP 5:将x_near和x_new之间的路径画出来
    hold on;
    plot(x_new(1), x_new(2), 'go', 'MarkerSize',4, 'MarkerFaceColor','g');
    
    % STEP 6:检查是否到达目标点附近 
    if norm(x_new - x_goal) < Thr
        bFind = true;
        break;
    end
    
    pause(0.05); %暂停一会，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询

if bFind
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end