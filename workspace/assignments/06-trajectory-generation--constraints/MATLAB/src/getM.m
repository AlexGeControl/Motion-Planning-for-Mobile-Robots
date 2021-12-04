function M = getM(K, t_order)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % num. of non-zero M elements:
    E = K*N*(N + 1)/2;

    % ###############################################
    % build M
    % ###############################################
    M_i = zeros(E, 1);
    M_j = zeros(E, 1);
    M_v = zeros(E, 1);
    b = zeros(N, 1);

    index = 1;

    for n = 1: N
        % binomial coefficient from bernstein polynomial:
        b(n) = nchoosek(N - 1, n - 1);
    end

    for n = 1: N
        for m = 1:n
            for k = 1:K
                M_i(index) = (k - 1)*N + n;
                M_j(index) = (k - 1)*N + m;
                % binomial coefficient from t^(i)*(1-t)^(n-i)
                M_v(index) = b(m) * nchoosek(N - m, n - m) * (-1)^(n - m);

                index = index + 1;
            end
        end
    end    
    
    % done:
    M = sparse(M_i, M_j, M_v);
end