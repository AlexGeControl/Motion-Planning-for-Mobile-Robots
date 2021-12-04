function [Aieq, bieq] = getAbieq(K, t_order, ts, corridor_range, v_max, a_max)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % num. of inequality constraints:
    D = K*(2*N - t_order + 1)*t_order;    

    % num. of non-zero Aieq elements:
    E = 0;
    for c=1:t_order
        E = E + (N - c + 1)*c;
    end
    E = 2*K*E;

    % ###############################################
    % pre-compute constants used in Aieq construction
    % ###############################################
    % factorial from derivative
    Aieq_factorial_k = [];
    Aieq_factorial_v = [];
    
    index = 1;
    
    for c = 1:t_order
        Aieq_factorial_k(index) = c;
        Aieq_factorial_v(index) = factorial(N - 1) / factorial(N - c);
            
        index = index + 1;
    end
    Aieq_factorial = containers.Map(Aieq_factorial_k, Aieq_factorial_v);

    % ###############################################
    % build constraint matrix
    % ###############################################
    index = 1;

    Aieq_i = zeros(E, 1);
    Aieq_j = zeros(E, 1);
    Aieq_v = zeros(E, 1);

    bieq = zeros(D, 1);

    c_index = 1;

    for c = 1:t_order
        for k = 1:K
            for n = c:N
                % set derivative:
                for i = 1:c
                    Aieq_i(index) = c_index;
                    Aieq_j(index) = ((k - 1)*N + n - i + 1);
                    Aieq_v(index) = Aieq_factorial(c) * nchoosek(c - 1, i - 1) * (-1)^(i - 1) / ts(k)^(c-1);

                    Aieq_i(index + 1) = c_index + 1;
                    Aieq_j(index + 1) = Aieq_j(index);
                    Aieq_v(index + 1) = -Aieq_v(index);

                    index = index + 2;
                end

                % set limit:
                if (c == 1)
                    bieq(c_index) = corridor_range(k, 2);
                    bieq(c_index + 1) = -corridor_range(k, 1);
                elseif (c == 2)
                    bieq(c_index) = v_max;
                    bieq(c_index + 1) = v_max;
                elseif (c == 3)
                    bieq(c_index) = a_max;
                    bieq(c_index + 1) = a_max;
                else 
                    bieq(c_index) = Inf;
                    bieq(c_index + 1) = Inf;
                end

                % move to next constraint:
                c_index = c_index + 2;
            end
        end
    end

    % done:
    Aieq = sparse(Aieq_i, Aieq_j, Aieq_v);
end