function [p_0,v_0,a_0] = forward(p_0,v_0,a_0,j,dt)
    % pos:
    p_0 = p_0 + v_0*dt + 0.5*a_0*dt^2 + 1/6*j*dt^3;
    % vel:
    v_0 = v_0 + a_0*dt + 0.5*j*dt^2;
    % acc:
    a_0 = a_0 + j*dt;
end