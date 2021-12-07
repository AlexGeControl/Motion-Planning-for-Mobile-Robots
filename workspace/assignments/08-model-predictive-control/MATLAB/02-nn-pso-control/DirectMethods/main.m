clear all;
close all;
clc;

%==========================================================================
% set up all the control points
%==========================================================================

TN = 10000;

K = 25;
v_max = 1.5;
v_min = -1.5;
a_max = 1.0;
a_min = -1.0;
j_max = 5.0;
j_min = -5.0;

H = eye(K);
J2A=zeros(K);
J2V=zeros(K);
J2P=zeros(K);

for i = 1:K
    J2A(i,1:i) = ones(1,i);
end

for i = 1:K
    for j = 1:i
        J2V(i,j) = (i-j+0.5);
    end
end

for i = 1:K
    for j = 1:i
        J2P(i,j) = ((i-j+1)*(i-j)/2+1/6);
    end
end

% initial condition lists
for ctt=1:100
    %% Set initial and end conditions
    IV_L = zeros(TN, 2);
    IA_L = zeros(TN, 2);
    
    % end condition lists
    EP_L = zeros(TN, 2);
    EV_L = zeros(TN, 2);
    EA_L = zeros(TN, 2);
    
    % time lists
    h_L = zeros(TN, 1);
    J_L = zeros(TN, 1);
    u_L = zeros(K,2,TN);
    for i=1:TN
        a_0 = [RG(a_min,a_max) RG(a_min,a_max)];
        v_0 = [RG(v_min,v_max) RG(v_min,v_max)];
        p_0 = [RG(0,0) RG(0,0)];
        
        a_f = [RG(a_min,a_max) RG(a_min,a_max)];
        v_f = [RG(v_min,v_max) RG(v_min,v_max)];
        p_f = [RG(-10,10) RG(-10,10)];
        
        tic;
        [F,h,success,converge] = nonConvexBisection(H,J2A,J2V,J2P,K,p_0,v_0,a_0,p_f,v_f,a_f,v_max,v_min,a_max,a_min,j_max,j_min);
        toc;
        
        IV_L(i,:) = v_0;
        IA_L(i,:) = a_0;
        
        % end condition lists
        EP_L(i,:) = p_f;
        EV_L(i,:) = v_f;
        EA_L(i,:) = a_f;
        
        % time lists
        if(success)
            h_L(i) = h;
            J_L(i) = F.f;
            u_L(:,:,i)=F.u;
        else
            h_L(i) = inf;
            J_L(i) = inf;
        end
        disp(i)
    end
    %%
    name = strcat('Data',num2str(ctt));
    save(name,'IV_L', 'IA_L', 'EP_L', 'EV_L', 'EA_L', 'h_L', 'J_L', 'u_L');
end