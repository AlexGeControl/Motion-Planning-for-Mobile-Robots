function C = getC(K, t_order)
    % num. of decision variables:
    D = (K + 1)*t_order;
    
    C_i = [];
    C_j = [];
    C_v = [];
    
    index = 1;
    c_index = 1;
    
    % ###############################################
    % 1. select fixed variables
    % ###############################################
    for c = 1:t_order
        % first trajectory segment:
        C_i(index) = c;
        C_j(index) = c_index;
        C_v(index) = 1.0;
        index = index + 1;
        
        % move to next decision variable:
        c_index = c_index + 1;
        
        % last trajectory segment:
        C_i(index) = (K - 1)*(2*t_order) + t_order + c;
        C_j(index) = c_index;
        C_v(index) = 1.0;
        index = index + 1;
        
        % move to next decision variable:
        c_index = c_index + 1;
    end

    % ###############################################
    % 2. select free variables
    % ###############################################
    for c = 1:t_order
        for k = 1:(K - 1)
            % current trajectory segment end state:
            C_i(index) = (k - 1)*(2*t_order) + t_order + c;
            C_j(index) = c_index;
            C_v(index) = 1.0;
            index = index + 1;
        
            % should equal to next trajectory segment start state:
            C_i(index) = k*(2*t_order) + c;
            C_j(index) = c_index;
            C_v(index) = 1.0;
            index = index + 1;
        
            % move to next decision variable:
            c_index = c_index + 1;
        end
    end
    
    C = sparse(C_i, C_j, C_v);
end