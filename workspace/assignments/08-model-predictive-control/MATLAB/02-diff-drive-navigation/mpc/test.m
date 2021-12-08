%contact: Lai Shupeng
%email: shupenglai@gmail.com
clear;
close all;
clc;

K = 25;

v_max = 1.5;
v_min = -1.5;
a_max = 1.0;
a_min = -1.0;
j_max = 2.0;
j_min = -2.0;

a_0 = 1;
v_0 = 3;

a_f = 0;
v_f = -3;

% a_0 = [RG(a_min,a_max) RG(a_min,a_max)];
% v_0 = [RG(v_min,v_max) RG(v_min,v_max)];
% p_0 = [RG(0,0) RG(0,0)];
% 
% a_f = [RG(a_min,a_max) RG(a_min,a_max)];
% v_f = [RG(v_min,v_max) RG(v_min,v_max)];
% p_f = [RG(10,10) RG(10,10)];

V = zeros(K,1);

H = eye(K);
J2A=zeros(K);
J2V=zeros(K);

for i = 1:K
    J2A(i,1:i) = ones(1,i);
end

for i = 1:K
    for j = 1:i
        J2V(i,j) = (i-j+0.5);
    end
end

tic;
[F,h,success,converge] = nonConvexBisection(H,J2A,J2V,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);
converge
toc;



if(success == 1)
    u=F.u;
    J2Ap = J2A*h;
    J2Vp = J2V*h^2;
    for m = 1:1
        V = zeros(K,1);
        P = zeros(K,1);
        A = a_0(m) + J2Ap*u(:,m);
        
        V_pr = J2Vp*u(:,m);
        for i=1:K
            V(i) = V_pr(i) + i*h*a_0(m) + v_0(m);
        end
        
        A=[a_0(m);A];
        V=[v_0(m);V];

        [vL(:,m),aL(:,m),tL(:,m)] = genSmoothTrajectory(V,A,u(:,m),K,h);
    end
    T=[0:K]*h;
%     plot(pL(:,1),pL(:,2));axis square;
        figure(1);
        subplot(3,1,1);plot(T,V,'x');
        hold on;plot(tL,vL);
        subplot(3,1,2);plot(T,A,'x');
        hold on;plot(tL,aL);
        subplot(3,1,3);stairs(u);
    F.f
    %     %check the derivatives
    %     dv = (pL(2:end)-pL(1:end-1))/0.02;
    %     da = (vL(2:end)-vL(1:end-1))/0.02;
    %     figure(2);
    %     subplot(2,1,1);hold on;plot(vL);plot(dv);
    %     subplot(2,1,2);hold on;plot(aL);plot(da);
else
    disp('No solution found.');
end
% 
% 
