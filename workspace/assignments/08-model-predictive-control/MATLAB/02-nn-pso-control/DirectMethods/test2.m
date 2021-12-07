%contact: Lai Shupeng
%email: shupenglai@gmail.com
clear;
close all;
clc;

K = 50;
h = 0.5;

v_max = 1.5;
v_min = -1.5;
a_max = 1.0;
a_min = -1.0;
j_max = 5.0;
j_min = -5.0;

a_0 =-0.5;
v_0 = 0.23;
p_0 = 0;

a_f = 0.123;
v_f = -0.9;
p_f = -3.612538049263366;

% a_0 =0.579695117369631;
% v_0 = 0.198759379826964;
% p_0 = 0;
% 
% a_f = 0.643172231023066;
% v_f = -0.585357511480487;
% p_f = -3.612538049263366;

% a_0 = [RG(a_min,a_max)];
% v_0 = [RG(v_min,v_max)];
% p_0 = [RG(0,0)];
% 
% a_f = [RG(a_min,a_max)];
% v_f = [RG(v_min,v_max)];
% p_f = [RG(-10,10)];

V = zeros(K,1);
P = zeros(K,1);

H = eye(K);
J2Ai=zeros(K);
J2Vi=zeros(K);
J2Pi=zeros(K);

for i = 1:K
    J2Ai(i,1:i) = ones(1,i);
end

for i = 1:K
    for j = 1:i
        J2Vi(i,j) = (i-j+0.5);
    end
end

for i = 1:K
    for j = 1:i
        J2Pi(i,j) = ((i-j+1)*(i-j)/2+1/6);
    end
end



[ u_qp,f,success ] = directSolve(H,J2Ai,J2Vi,J2Pi,h,K,p_0,v_0,a_0,p_f,v_f,a_f,v_max,v_min,a_max,a_min,j_max,j_min);
success
if(success == 1)
    u0=[u_qp;h];
else
    u0=[zeros(K,1); h];
end

ub = [ones(K,1)*j_max;2];
lb = [ones(K,1)*j_min;0.02];


options = optimoptions('fmincon','Display','final','Algorithm','sqp','MaxFunEvals',4000,'TolFun',5e-2,'TolCon',5e-2,'GradObj','on','GradConstr','on');
tic;
[u,fval,exitflag,output] = fmincon(@(x)myfun(x,K),u0,[],[],[],[],lb,ub,@(x)mycon(x,J2Ai,J2Vi,J2Pi,K,p_0,v_0,a_0,p_f,v_f,a_f,v_max,v_min,a_max,a_min),options);
toc;
fval

if(exitflag==1 || exitflag==2)
        J2A = J2Ai*u(K+1);
        J2V = J2Vi*u(K+1)^2;
        J2P = J2Pi*u(K+1)^3;
        for m = 1:1
            V = zeros(K,1);
            P = zeros(K,1);
            A = a_0(m) + J2A*u(1:K,m);
            
            V_pr = J2V*u(1:K,m);
            for i=1:K
                V(i) = V_pr(i) + i*u(K+1)*a_0(m) + v_0(m);
            end
            
            P_pr = J2P*u(1:K,m);
            for i=1:K
                P(i)=i*u(K+1)*v_0(m)+p_0(m)+i^2/2*u(K+1)^2*a_0(m)+P_pr(i);
            end
            A=[a_0(m);A];
            V=[v_0(m);V];
            P=[p_0(m);P];
            [pL(:,m),vL(:,m),aL(:,m),tL(:,m)] = genSmoothTrajectory(P,V,A,u(1:K,m),K,u(K+1));
        end
        T = [0:1:K]*u(K+1);
end
%     J2Ap = J2A*h;
%     J2Vp = J2V*h^2;
%     J2Pp = J2P*h^3;
%     for m = 1:2
%         V = zeros(K,1);
%         P = zeros(K,1);
%         A = a_0(m) + J2Ap*u(:,m);
%
%         V_pr = J2Vp*u(:,m);
%         for i=1:K
%             V(i) = V_pr(i) + i*h*a_0(m) + v_0(m);
%         end
%
%         P_pr = J2Pp*u(:,m);
%         for i=1:K
%             P(i)=i*h*v_0(m)+p_0(m)+i^2/2*h^2*a_0(m)+P_pr(i);
%         end
%         A=[a_0(m);A];
%         V=[v_0(m);V];
%         P=[p_0(m);P];
%         [pL(:,m),vL(:,m),aL(:,m),tL(:,m)] = genSmoothTrajectory(P,V,A,u(:,m),K,h);
%     end
%     plot(pL(:,1),pL(:,2));axis square;
    figure(1);
    subplot(4,1,1);plot(T,P,'x');
    hold on;plot(tL,pL);
    subplot(4,1,2);plot(T,V,'x');
    hold on;plot(tL,vL);
    subplot(4,1,3);plot(T,A,'x');
    hold on;plot(tL,aL);
    subplot(4,1,4);stairs(u);
%     sum(f)

%     %check the derivatives
%     dv = (pL(2:end)-pL(1:end-1))/0.02;
%     da = (vL(2:end)-vL(1:end-1))/0.02;
%     figure(2);
%     subplot(2,1,1);hold on;plot(vL);plot(dv);
%     %     subplot(2,1,2);hold on;plot(aL);plot(da);
% else
%     disp('No solution found.');
% end

% clear;
% close all;
% clc;
%
% x0=[zeros(25,1); 0.5];
% tic;
% options = optimoptions('fmincon','Display','final','Algorithm','sqp');
% [x,fval,exitflag,output] = fmincon(@myfun,x0,[],[],[],[],[],[],@mycon,options);
% toc;
% x(26)
