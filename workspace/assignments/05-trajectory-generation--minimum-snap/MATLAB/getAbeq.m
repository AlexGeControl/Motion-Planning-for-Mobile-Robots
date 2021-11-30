function [Aeq, beq]= getAbeq(K, t_order, waypoints, ts, start_cond, end_cond)    
    % num. of polynomial coeffs:
    N = 2*t_order;
    
    % num. of constraints:
    C = (K + 1)*t_order + (K - 1);
    
    % ###############################################
    % 1. pre-compute constants used in A construction
    % ###############################################
    % 1.1 factorial from derivative
    A_factorial_k = [];
    A_factorial_v = [];
    
    index = 1;
    
    for c = 1:t_order
        for n = c:N
            A_factorial_k(index) = (c-1)*N + n - 1;
            A_factorial_v(index) = factorial(n - 1) / factorial(n - c);
            
            index = index + 1;
        end
    end
    A_factorial = containers.Map(A_factorial_k, A_factorial_v);
    
    % ###############################################
    % 2. populate A & b
    % ###############################################
    A_i = [];
    A_j = [];
    A_v = [];
    
    beq = zeros(C, 1);
    
    index = 1;
    c_index = 1;
    
    % 2.1 start & goal states:
    for c = 1:t_order
        % start state:
        A_i(index) = c_index;
        A_j(index) = c;
        A_v(index) = A_factorial((c-1)*N + c - 1)/(ts(1)^(c - 1));
        index = index + 1;
        
        beq(c_index) = start_cond(c);
        
        % move to next constraint:
        c_index = c_index + 1;
        
        % end state:
        for n = c:N
            A_i(index) = c_index;
            A_j(index) = (K - 1)*N + n;
            A_v(index) = A_factorial((c-1)*N + n - 1)/(ts(K)^(c - 1));
            index = index + 1;
        end
        
        beq(c_index) = end_cond(c);
        
        % move to next constraint:
        c_index = c_index + 1;
    end
    
    % 2.2 intermediate waypoint passing constraints:
    for k = 1:(K - 1)
        % next segment start position:
        A_i(index) = c_index;
        A_j(index) = k*N + 1;
        A_v(index) = 1.0;
        index = index + 1;
        
        % should equal to the specified value:
        beq(c_index) = waypoints(k+1);
        
        % move to next constraint:
        c_index = c_index + 1;
    end
    
    % 2.3 intermediate waypoint continuity constraints:
    for c = 1:t_order
        for k = 1:(K - 1)
            % current segment end state:
            for n = c:N
                A_i(index) = c_index;
                A_j(index) = (k - 1)*N + n;
                A_v(index) = A_factorial((c-1)*N + n - 1)/(ts(k)^(c - 1));
                index = index + 1;
            end
            
            % should equal to next segment start state:
            A_i(index) = c_index;
            A_j(index) = k*N + c;
            A_v(index) = -A_factorial((c-1)*N + c - 1)/(ts(k + 1)^(c - 1));
            index = index + 1;
            
            % move to next constraint:
            c_index = c_index + 1;
        end
    end
    
    Aeq = sparse(A_i, A_j, A_v);
end