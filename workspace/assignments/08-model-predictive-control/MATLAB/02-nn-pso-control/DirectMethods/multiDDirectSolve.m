function F = multiDDirectSolve(H,J2A,J2V,h,K,...
    v_0,a_0,v_f,a_f,...
    a_max,a_min,j_max,j_min)
N = length(v_0);
f = 0;
F.u = zeros(K,N);
F.success=1;
for i=1:N
    [ u_tmp,f_tmp,success] = directSolve(H,J2A,J2V,h,K,...
        v_0(i),a_0(i),v_f(i),a_f(i),...
        a_max,a_min,j_max,j_min);
    if (success ~= 1)
        F.u=[];
        F.f=inf;
        F.success = 0;
        return;
    else
        F.u(:,i)=u_tmp;
        f = f + f_tmp;
    end
end

F.f=f+1*K*h;


end