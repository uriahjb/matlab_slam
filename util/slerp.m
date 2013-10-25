%{
    Perform spherical linear interpolation between quaternions
    This is pretty awesome. Apparently there are other types 
    of quaternion interpolation that might be worth checking
    out. 
%}
function q_new = slerp(q0,q1,t)
    theta_thresh = 1e-8;
    if t < 0 || t > 1.0
        disp('t out of range');
        return
    end
    % compute q^t
    theta = 2*acos(q0'*q1);
    if abs(theta) < theta_thresh
        % Can't decide if its justified to average the quaternions in this
        % case ... probably not
        q_new = q0;
        return
    end
    q_new = (q0.*sin((1-t)*theta) + q1.*sin(t*theta))/sin(theta);
    q_new = q_new/qnorm(q_new);
end