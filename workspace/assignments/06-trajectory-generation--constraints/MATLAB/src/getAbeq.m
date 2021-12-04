function [Aeq, beq] = getAbeq(K, t_order, ts, start_cond, end_cond)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % num. of equality constraints:
    D = (K + 1)*t_order;

    % num. of non-zero Aieq elements:
    E = K*(t_order + 1)*t_order;

    % ###############################################
    % pre-compute constants used in A construction
    % ###############################################
    % factorial from derivative
    Aeq_factorial_k = [];
    Aeq_factorial_v = [];
    
    index = 1;
    
    for c = 1:t_order
        Aeq_factorial_k(index) = c;
        Aeq_factorial_v(index) = factorial(N - 1) / factorial(N - c);
            
        index = index + 1;
    end
    Aeq_factorial = containers.Map(Aeq_factorial_k, Aeq_factorial_v);

    % ###############################################
    % build constraint matrix
    % ###############################################
    index = 1;

    Aeq_i = zeros(E, 1);
    Aeq_j = zeros(E, 1);
    Aeq_v = zeros(E, 1);

    beq = zeros(D, 1);

    c_index = 1;

    % ###############################################
    % start & end conditions
    % ###############################################
    for c = 1:t_order
        for i = 1:c
            % start condition:
            Aeq_i(index) = c_index;
            Aeq_j(index) = i;
            Aeq_v(index) = Aeq_factorial(c) * nchoosek(c - 1, c - i) * (-1)^(c - i) / ts(1)^(c-1);
            
            % end condition:
            Aeq_i(index + 1) = c_index + 1;
            Aeq_j(index + 1) = (K - 1)*N + N - i + 1;
            Aeq_v(index + 1) = Aeq_factorial(c) * nchoosek(c - 1, i - 1) * (-1)^(i - 1) / ts(K)^(c-1);

            index = index + 2;
        end
        
        % start condition:
        beq(c_index) = start_cond(c);
        % end condition:
        beq(c_index + 1) = end_cond(c);
        
        % move to next constraint:
        c_index = c_index + 2;
    end

    % ###############################################
    % intermediate waypoint continuity constraints
    % ###############################################
    for k = 1:(K - 1)
        for c = 1:t_order
            for i = 1:c
                % end state of current trajectory segment:
                Aeq_i(index) = c_index;
                Aeq_j(index) = (k - 1)*N + N - i + 1;
                Aeq_v(index) = Aeq_factorial(c) * nchoosek(c - 1, i - 1) * (-1)^(i - 1) / ts(k)^(c-1);

                % should equal to start state of next trajectory segment:
                Aeq_i(index + 1) = c_index;
                Aeq_j(index + 1) = k * N + i;
                Aeq_v(index + 1) = -Aeq_factorial(c) * nchoosek(c - 1, c - i) * (-1)^(c - i) / ts(k + 1)^(c-1);

                index = index + 2;
            end

            % move to next constraint:
            c_index = c_index + 1;
        end
    end

    % done:
    Aeq = sparse(Aeq_i, Aeq_j, Aeq_v);
end