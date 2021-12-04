function [Aeq, beq] = getAbeq(K, t_order, ts, start_cond, end_cond)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % num. of equality constraints:
    D = (K + 1)*t_order;

    % ###############################################
    % pre-compute constants used in A construction
    % ###############################################
    % factorial from derivative
    A_factorial_k = [];
    A_factorial_v = [];
    
    index = 1;
    
    for c = 1:t_order
        A_factorial_k(index) = c;
        A_factorial_v(index) = factorial(N - 1) / factorial(N - c);
            
        index = index + 1;
    end
    A_factorial = containers.Map(A_factorial_k, A_factorial_v);

    Aeq = zeros(D, K*N);
    beq = zeros(D, 1);

    c_index = 1;

    % ###############################################
    % boundary constraints
    % ###############################################
    for c = 1:t_order
        % start conditions:
        for i = 1:c
            Aeq(c_index, i) = nchoosek(c - 1, c - i) * (-1)^(c - i);
        end
        Aeq(c_index, :) = A_factorial(c) / ts(1)^(c-1) * Aeq(c_index, :);
        beq(c_index) = start_cond(c);
        % move to next constraint:
        c_index = c_index + 1;

        % end conditions:
        for i = 1:c
            Aeq(c_index, (K - 1)*N + N - i + 1) = nchoosek(c - 1, i - 1) * (-1)^(i - 1);
        end
        Aeq(c_index, :) = A_factorial(c) / ts(K)^(c-1) * Aeq(c_index, :);
        beq(c_index) = end_cond(c);
        % move to next constraint:
        c_index = c_index + 1;
    end

    % ###############################################
    % intermediate waypoint continuity constraints
    % ###############################################
    for k = 1:(K - 1)
        for c = 1:t_order
            % end state of current trajectory segment:
            for i = 1:c
                Aeq(c_index, (k - 1)*N + N - i + 1) = nchoosek(c - 1, i - 1) * (-1)^(i - 1) / ts(k)^(c-1);
            end

            % should equal to start state of next trajectory segment:
            for i = 1:c
                Aeq(c_index, k * N + i) = -nchoosek(c - 1, c - i) * (-1)^(c - i) / ts(k + 1)^(c-1);
            end

            % multiply by derivative factorial:
            Aeq(c_index, :) = A_factorial(c) * Aeq(c_index, :);

            % move to next constraint:
            c_index = c_index + 1;
        end
    end
end