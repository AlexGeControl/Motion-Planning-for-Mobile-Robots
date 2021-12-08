%contact: Lai Shupeng
%email: shupenglai@gmail.com
function [vL,aL,tL] = genSmoothTrajectory(V,A,u,K,h)
dt = 0.02;
vL=[];aL=[];
tL=0:dt:K*h;
for t=0:dt:K*h
    s = ceil(t/h);
    if(s==0)
       s=1; 
    end
    st = t - (s-1)*h;
    a = A(s)+st*u(s);
    v = V(s)+st*A(s)+0.5*st^2*u(s);

    vL=[vL;v];
    aL=[aL;a];
end
end