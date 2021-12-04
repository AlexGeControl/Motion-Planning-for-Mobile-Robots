function M = getM(t_order)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % ###############################################
    % build M
    % ###############################################
    M_i = [];
    M_j = [];
    M_v = [];
    b = zeros(N, 1);

    index = 1;

    for n = 1: N
        % binomial coefficient from bernstein polynomial:
        b(n) = nchoosek(N - 1, n - 1);

        % binomial coefficient from t^(i)*(1-t)^(n-i)
        for m = 1:n
            M_i(index) = n;
            M_j(index) = m;
            M_v(index) = nchoosek(N - m, n - m) * (-1)^(n - m);

            index = index + 1;
        end
    end
    
    M = sparse(M_i, M_j, M_v);

    % done:
    M = M * diag(b);
end