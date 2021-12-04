function [Aieq, bieq] = getAbieq(K, t_order, ts, corridor_range, v_max, a_max)
    % num. of polynomial coeffs: 
    N = 2*t_order;

    % num. of inequality constraints:
    D = K*(2*N - t_order + 1)*t_order;    

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

    Aieq = zeros(D, K*N);
    bieq = zeros(D, 1);

    c_index = 1;

    for c = 1:3
        for k = 1:K
            % set upper bound:
            for n = c:N
                for i = 1:c
                    Aieq(c_index, (k - 1)*N + n - i + 1) = nchoosek(c - 1, i - 1) * (-1)^(i - 1) / ts(k)^(c-1);
                end
                % multiply by derivative factorial:
                Aieq(c_index, :) = A_factorial(c) * Aieq(c_index, :);

                % set limit:
                if (c == 1)
                    bieq(c_index) = corridor_range(k, 2);
                elseif (c == 2)
                    bieq(c_index) = v_max;
                else
                    bieq(c_index) = a_max;
                end

                % move to next constraint:
                c_index = c_index + 1;

                Aieq(c_index, :) = -Aieq(c_index - 1, :);
                % set limit:
                if (c == 1)
                    bieq(c_index) = -corridor_range(k, 1);
                elseif (c == 2)
                    bieq(c_index) = v_max;
                else
                    bieq(c_index) = a_max;
                end

                % move to next constraint:
                c_index = c_index + 1;
            end
        end
    end
end