function [P, M] = getPM(K, t_order, ts)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % ###############################################
    % get Q, objective matrix for minimum snap
    % ###############################################
    Q = getQ(K, t_order, ts);
    % ###############################################
    % get M, mapping from bernstein to monomial
    % ###############################################
    M = getM(K, t_order);
    % ###############################################
    % build objective matrix for control points
    % ###############################################
    % P = nearestSPD(full(M'*Q*M));
    P = M'*Q*M;
end