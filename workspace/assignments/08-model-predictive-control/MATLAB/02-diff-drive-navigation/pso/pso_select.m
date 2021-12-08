function [theta_target, v_target] = pso_select(theta,t,omega,v,last_theta_target,last_v_target)
    R=[cos(theta), -sin(theta); sin(theta),  cos(theta)];

    %==========================================================================
    % algo. configuration
    %==========================================================================
    N     = 10;
    batch = 10;
    k1    = 1.0;
    k2    = 1.0;

    %==========================================================================
    % init particles
    %==========================================================================
    % each particle is defined as: [theta, v_end, v_theta, v_vend, best_theta, best_v, best_cost]
    particle = zeros(N,7);
    global_best = zeros(1,3);
    global_best(1) = last_theta_target-theta;
    global_best(2) = last_v_target;
    global_best(3) = evaluate(R,t,omega,last_theta_target-theta,last_theta_target-theta,v,last_v_target);
    for i=1:N
        particle(i,1)=(rand-0.5)*1.8*pi;
        particle(i,2)=(rand-0.5)*4+2;
        particle(i,3)=rand-0.5;
        particle(i,4)=rand-0.5;
        particle(i,5)=particle(i,1);
        particle(i,6)=particle(i,2);
        particle(i,7)=inf;
    end

    %==========================================================================
    % do PSO
    %==========================================================================
    for j=1:batch
        for i=1:N
            w = 0.95 - (0.95-0.4)/batch*j;

            if (j~=1)
                % get delta:
                delta_ith_best    = particle(i,5:6) - particle(i,1:2);
                delta_global_best = global_best(1:2) - particle(i,1:2);
                particle(i,3:4)   = particle(i,3:4) + k1*rand*delta_ith_best + k2*rand*delta_global_best;
                
                % update state
                particle(i,1:2)   = particle(i,1:2) + particle(i,3:4);
                particle(i,1:2)   = limit_range(particle(i,1:2));
            end
            
            % evaluate the particle:
            cost = evaluate(R,t,omega,particle(i,1),last_theta_target-theta,v,particle(i,2));
            
            % update the local best
            if cost < particle(i,7)
                particle(i,7) = cost;
                particle(i,5)=particle(i,1);
                particle(i,6)=particle(i,2);
            end
            
            % update the global best
            if cost < global_best(3)
                global_best(3)=cost;
                global_best(1)=particle(i,1);
                global_best(2)=particle(i,2);
            end
        end
    end
    global_best(1) = global_best(1) + theta;

    %==========================================================================
    % done
    %==========================================================================
    theta_target = global_best(1);
    v_target     = global_best(2);
end