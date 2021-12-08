function cost = evaluate(R,t,omega,delta_theta,last_delta_theta,v_ini,v_end)
    global hMap

    %==========================================================================
    % algo. configuration
    %==========================================================================
    N             = 40;

    w_delta_theta = 8;
    w_v           = 8;

    %==========================================================================
    % get evaluation points
    %==========================================================================
    P = forward_unic([omega, delta_theta,v_ini,v_end]');
    P = R*[P(1:N) P(N + 1:2*N)]';
    X = [P(1,:)]'+ t(1);
    Y = [P(2,:)]'+ t(2);

    cost = 0;
    grid = zeros(1,2);
    for i=1:N
        % transform position to map grid:
        grid(1) = round(3*(X(i)+10)+1);
        grid(2) = round(3*(Y(i)+10)+1);

        % get score, position:
        if grid(1) > 0 && grid(1) < 640 && grid(2) >0 && grid(2) < 640
            cost = cost + hMap(grid(1), grid(2));
        else
            cost = cost + inf;
        end
    end

    cost = cost + w_delta_theta*norm(delta_theta-last_delta_theta) + w_v*norm(v_end-v_ini);
end