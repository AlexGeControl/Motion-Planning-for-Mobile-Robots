function [P, M] = getP(K, t_order, ts)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % ###############################################
    % get Q, objective matrix for minimum snap
    % ###############################################
    Q = getQ(K, t_order, ts);
    % ###############################################
    % get M, mapping from bernstein to monomial
    % ###############################################
    M_elem = getM(t_order);
    M = zeros(K*N, K*N);
    for k=1:K
        segment_index = ((k - 1)*N + 1):(k*N);
        M(segment_index,segment_index) = M_elem;
    end
    % ###############################################
    % build objective matrix for control points
    % ###############################################
    P = nearestSPD(M'*Q*M);
end