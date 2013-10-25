function [ang] = q2ang( q )
    if (q(1) > 1.0)
        q = q/qnorm(q);
    end
    
    
    ang = zeros(3,1);
    %alpha = 2*acos(q(1));
    alpha = 2*atan2(norm(q(2:4)), q(1));
    
    if alpha > pi
        alpha = alpha - 2*pi;
        %alpha = -alpha;
    end
    
    
    if abs(alpha) > pi
        error('alpha to great');
    end
    
    %{
    if alpha < 1e-16 && alpha > -1e-16
        ang = [0 0 0]';
        return
    end
    %}
    if abs(1-q(1)) < 1e-16
        ang = [0 0 0]';
        return
    end
    
    %axis = alpha*q(2:4)/sin(alpha/2);
    axis = alpha*q(2:4)/sqrt(1-q(1)^2);
    ang = axis;
    
end