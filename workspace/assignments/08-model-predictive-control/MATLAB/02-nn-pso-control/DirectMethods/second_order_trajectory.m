function [success,vL,aL,h]=second_order_trajectory(a_0,v_f)

K = 25;

a_max = 1.0;
a_min = -1.0;
j_max = 2.0;
j_min = -2.0;

v_0 = 0;
a_f = 0;


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

[F,h,success,converge] = nonConvexBisection(H,J2A,J2V,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);



vL=[];
if(success == 1)
    u=F.u;
    J2Ap = J2A*h;
    J2Vp = J2V*h^2;
    for m = 1:1
        V = zeros(K,1);
        A = a_0 + J2Ap*u;
        
        V_pr = J2Vp*u;
        for i=1:K
            V(i) = V_pr(i) + i*h*a_0 + v_0;
        end
        
        A=[a_0;A];
        V=[v_0;V];

        [vL,aL,tL] = genSmoothTrajectory(V,A,u(:,m),K,h);
    end
end


end