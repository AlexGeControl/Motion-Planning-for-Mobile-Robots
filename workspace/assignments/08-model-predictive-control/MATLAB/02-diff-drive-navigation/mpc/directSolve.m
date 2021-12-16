function [u, cost, success] = directSolve(H,J2A,J2V,h,K,v_0,a_0,v_f,a_f,a_max,a_min,j_max,j_min)
    %==========================================================================
    % control matrix of zero-order hold jerks
    %==========================================================================
    J2V = J2V*h^2;
    J2A = J2A*h;

    %==========================================================================
    % at timestamp = K*h, ego vehicle state (v, a) should equal to (v_f, a_f)
    %==========================================================================
    Aeq = [J2V(K,:); J2A(K,:)];
    beq = [v_f-v_0-K*h*a_0; a_f-a_0];

    %==========================================================================
    % acc. magnitude should stay within the bound [a_min, a_max]
    %==========================================================================
    Aieq = [J2A;-J2A];
    bieq = ones(2*K,1);
    for i=1:K
        bieq(i) = a_max - a_0;
    end
    for i=K+1:2*K
        bieq(i) = -a_min + a_0;
    end

    %==========================================================================
    % jerk magnitude should stay within the bound [j_min, j_max]
    %==========================================================================
    ub = ones(K,1)*j_max;
    lb = ones(K,1)*j_min;

    %==========================================================================
    % solve linear MPC with objective function minimum jerk
    %==========================================================================
    options = optimoptions('quadprog', 'Display', 'off');
    f = zeros(length(H),1);
    [u, cost, success] = quadprog(H, f, Aieq, bieq, Aeq, beq, lb, ub, [], options);

    cost = cost*h;
end

