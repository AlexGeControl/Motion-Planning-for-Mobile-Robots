%contact: Lai Shupeng
%email: shupenglai@gmail.com
function [ u,f,success ] = directSolve(H,J2A,J2V,h,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min)
J2A = J2A*h;
J2V = J2V*h^2;
Aeq = [J2V(K,:); J2A(K,:)];
beq = [v_f-v_0-K*h*a_0; a_f-a_0];

Aieq = [J2A;-J2A];
bieq = ones(2*K,1);
for i=1:K
    bieq(i) = a_max - a_0;
end
for i=K+1:2*K
    bieq(i) = -a_min + a_0;
end

ub = ones(K,1)*j_max;
lb = ones(K,1)*j_min;
% options = optimoptions('quadprog','Algorithm','interior-point-convex',...
%     'Display','off');
Fb=zeros(length(H),1);
[u,f,success] = quadprog(H,Fb,Aieq,bieq,Aeq,beq,lb,ub,[],[]);
f=f*h^2;
end

