function delta=limit_range(delta)
    if delta(1) > 0.9*pi
        delta(1) = 0.9*pi;
    end
    if delta(1) < -0.9*pi
        delta(1) = -0.9*pi;
    end

    if delta(2) > 4
        delta(2) = 4;
    end
    if delta(2) < -0.5
        delta(2) = -0.5;
    end
end