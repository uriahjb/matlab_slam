function rot = q2r( q )
    q0 = q(1); % s
    q1 = q(2); % x
    q2 = q(3); % y
    q3 = q(4); % z
    
    % Double check
    rot = [ (q0^2+q1^2-q2^2-q3^2) (-2*q0*q3+2*q1*q2) (2*q0*q2+2*q1*q3); ...
            (2*q0*q3+2*q1*q2)  (q0^2-q1^2+q2^2-q3^2) (-2*q0*q1+2*q2*q3); ...
            (-2*q0*q2+2*q1*q3) (2*q0*q1+2*q2*q3) (q0^2-q1^2-q2^2+q3^2) ];
end