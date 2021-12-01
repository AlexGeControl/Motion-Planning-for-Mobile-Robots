function M = getM(K, t_order, ts)
    % num. of polynomial coeffs:
    N = 2*t_order;
        
    % ###############################################
    % 1. pre-compute constants used in M construction
    % ###############################################
    % 1.1 factorial from derivative
    M_factorial_k = [];
    M_factorial_v = [];
    
    index = 1;
    
    for c = 1:t_order
        for n = c:N
            M_factorial_k(index) = (c-1)*N + n - 1;
            M_factorial_v(index) = factorial(n - 1) / factorial(n - c);
            
            index = index + 1;
        end
    end
    M_factorial = containers.Map(M_factorial_k, M_factorial_v);
    
    % ###############################################
    % 2. populate M
    % ###############################################
    M_i = [];
    M_j = [];
    M_v = [];
    
    index = 1;
    c_index = 1;
        
    % calculate the start & end states of each trajectory segment:
    for k = 1:K
        for c = 1:t_order
            % current segment start state:
            M_i(index) = c_index;
            M_j(index) = (k-1)*N + c;
            M_v(index) = M_factorial((c-1)*N + c - 1)/(ts(k)^(c - 1));
            index = index + 1;
            % move to next constraint:
            c_index = c_index + 1;
        end
        
        for c = 1:t_order
            % current segment end state:
            for n = c:N
                M_i(index) = c_index;
                M_j(index) = (k-1)*N + n;
                M_v(index) = M_factorial((c-1)*N + n - 1)/(ts(k)^(c - 1));
                index = index + 1;
            end
            % move to next constraint:
            c_index = c_index + 1;
        end
    end
    
    M = sparse(M_i, M_j, M_v);
end