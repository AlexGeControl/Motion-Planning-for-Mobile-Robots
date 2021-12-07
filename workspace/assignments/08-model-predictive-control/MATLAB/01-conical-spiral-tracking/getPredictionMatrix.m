function [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0)
    % ###############################################
    % system state under zero-order hold jerk control
    % ###############################################

    % zero-state response:
    Ta=zeros(K);
    Tv=zeros(K);
    Tp=zeros(K);
    for i = 1:K         
        for j = 1:i
            duration = (i-j);

            Ta(i,j) = dt;
            Tv(i,j) = 1/2*(2*duration+1)*(dt^2);
            Tp(i,j) = 1/6*(3*duration*(duration +1) + 1)*(dt^3);
        end
    end

    % zero-input response:
    Ba = ones(K,1)*a_0;
    Bv = ones(K,1)*v_0;
    Bp = ones(K,1)*p_0;
    for i=1:K
        Bv(i) = Bv(i) + a_0*i*dt;
        Bp(i) = Bp(i) + v_0*i*dt + a_0*1/2*(i^2)*dt^2;
    end
end