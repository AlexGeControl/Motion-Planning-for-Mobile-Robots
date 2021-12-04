function Q = getQ(K, t_order, ts)
    % num. of polynomial coeffs: 
    N = 2*t_order;
    
    % ###############################################
    % pre-compute constants used in Q construction
    % ###############################################
    % factorial from derivative
    Q_k = zeros(N - t_order);
    Q_v = zeros(N - t_order);
    for n = t_order:(N - 1)
        Q_k(n - t_order + 1) = n;
        Q_v(n - t_order + 1) = factorial(n) / factorial(n - t_order);
    end
    Q_factorial = containers.Map(Q_k, Q_v);
    
    % time power:
    ts_power = ts .^ t_order;

    % ###############################################
    % populate Q
    % ###############################################
    Q_i = [];
    Q_j = [];
    Q_v = [];
    
    index = 1;
    
    for k = 1:K
        for m = t_order:(N - 1)
            for n = t_order:(N - 1)
                Q_i(index) = (k - 1)*N + m + 1;
                Q_j(index) = (k - 1)*N + n + 1;
                Q_v(index) = ts(k)*Q_factorial(m)*Q_factorial(n)/((m + n - 2*t_order + 1)*ts_power(k)*ts_power(k));
                
                index = index + 1;
            end
        end
    end
    
    Q = sparse(Q_i, Q_j, Q_v);
end