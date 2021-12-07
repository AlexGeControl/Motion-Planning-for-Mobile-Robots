function [F_best,h_best,success,converge] = nonConvexBisection(H,J2A,J2V,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min)
success=1;
converge=0;
F_best=[];
h_best=[];

h_a = 0.02;
h_b = 2;

h_m = 0.5*(h_a + h_b);
F_a = multiDDirectSolve(H,J2A,J2V,h_a,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);
F_b = multiDDirectSolve(H,J2A,J2V,h_b,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);
F_m = multiDDirectSolve(H,J2A,J2V,h_m,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);


% Binary section search
%--------------------------------------------------------------------------
for i=1:12
    if (abs(h_a-h_b)<0.02)
        converge=1;
        break;
    end
    h_l = 0.5*(h_a + h_m);
    h_r = 0.5*(h_m + h_b);
    F_l = multiDDirectSolve(H,J2A,J2V,h_l,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);
    F_r = multiDDirectSolve(H,J2A,J2V,h_r,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);
    [~, F_min_i] = min([F_a.f, F_b.f, F_m.f, F_l.f, F_r.f]);
    
    % If all case ends up with no solution, we deduct that the state
    % constraints have to be violated due to the initial conditions
    %----------------------------------------------------------------------
    if(F_a.success ~= 1 && F_b.success ~= 1 && F_m.success ~= 1 && F_l.success ~= 1 && F_r.success ~=1)
        success = 0;
        break;
    end
    
    switch F_min_i
        case 1
            h_best = h_a;
        case 2
            h_best = h_b;
        case 3
            h_best = h_m;
        case 4
            h_best = h_l;
        case 5
            h_best = h_r;
    end
    
    if (F_min_i==1 || F_min_i==4)
        h_b = h_m;
        h_m = h_l;
        F_b = F_m;
        F_m = F_l;
    elseif (F_min_i==3)
        h_a = h_l;
        h_b = h_r;
        F_a = F_l;
        F_b = F_r;
    elseif (F_min_i==5 || F_min_i==2)
        h_a = h_m;
        h_m = h_r;
        F_a = F_m;
        F_m = F_r;
    end
end
%end repeat

% Generate the output based on whether there is a solution
%--------------------------------------------------------------------------
if success
    F_best=multiDDirectSolve(H,J2A,J2V,h_best,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min);
else
    F_best=[];
    h_best=[];
end
end
