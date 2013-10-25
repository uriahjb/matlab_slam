function q = ang2q( ang )

    angle = norm(ang);

    if angle < 1e-16
        axis = [0 0 0]';
    else
        axis = ang/angle;
    end
    q = [cos(angle/2); axis*sin(angle/2)]; 
end