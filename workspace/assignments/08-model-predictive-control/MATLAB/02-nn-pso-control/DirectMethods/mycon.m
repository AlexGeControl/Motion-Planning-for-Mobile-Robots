function [ c,ceq,DC, DCeq ] = mycon(x,J2Ai,J2Vi,J2Pi,K,p_0,v_0,a_0,p_f,v_f,a_f,v_max,v_min,a_max,a_min)
h = x(K+1);
J2A = J2Ai*h;
J2V = J2Vi*h^2;
J2P = J2Pi*h^3;

% ceq
%--------------------------------------------------------------------------
Aeq = [J2P(K,:); J2V(K,:); J2A(K,:)];
beq = [p_f-p_0-K*h*v_0-K^2/2*a_0*h^2 ; v_f-v_0-K*h*a_0; a_f-a_0];
ceq = Aeq*x(1:K)-beq;

% dceq
%--------------------------------------------------------------------------
DCeq(:,1)=[J2P(K,:)';3*J2Pi(K,:)*h^2*x(1:K)+K*v_0+K^2*a_0*h]; 
DCeq(:,2)=[J2V(K,:)';2*J2Vi(K,:)*h*x(1:K)+K*a_0]; 
DCeq(:,3)=[J2A(K,:)';1*J2Ai(K,:)*x(1:K)]; 

%c
%--------------------------------------------------------------------------
Aieq = [J2V;-J2V;J2A;-J2A];
bieq = ones(4*K,1);
for i=1:K
    bieq(i) = v_max - v_0 - i*h*a_0;
end
for i=K+1:2*K
    bieq(i) = -v_min + v_0 + (i-K)*h*a_0;
end
for i=2*K+1:3*K
    bieq(i) = a_max - a_0;
end
for i=3*K+1:4*K
    bieq(i) = -a_min + a_0;
end

c = Aieq*x(1:K)-bieq;

%dc
%--------------------------------------------------------------------------
DC=zeros(K+1,4*K);
for i=1:K
    DC(:,i)=[J2V(i,:)';2*J2Vi(i,:)*h*x(1:K)+a_0*i];
end
for i=K+1:2*K
    j=i-K;
    DC(:,i)=[-J2V(j,:)';-2*J2Vi(j,:)*h*x(1:K)-a_0*j];
end
for i=2*K+1:3*K
    j=i-2*K;
    DC(:,i)=[J2A(j,:)';J2Ai(j,:)*x(1:K)];
end
for i=3*K+1:4*K
    j=i-3*K;
    DC(:,i)=[-J2A(j,:)';-J2Ai(j,:)*x(1:K)];
end

