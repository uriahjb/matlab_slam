function [ypr] = rot2ypr( rot )
    % convert zyx rotation matrix to yaw pitch roll angles
    if rot(3,1) < 1 
        if rot(3,1) > -1
            y = atan2( rot(2,1), rot(1,1) );
            p = asin( -rot(3,1) );
            r = atan2( rot(3,2), rot(3,3) );
            ypr = [y;p;r];
            return
        else % r20 -1
            y = -atan2( -rot(2,3), rot(2,2) );
            p = pi/2;
            r = 0;
            ypr = [y;p;r];
            return
        end
    else % r20 +1
        y = atan2( -rot(2,3), rot(2,2) );
        p = pi/2;
        r = 0;
        ypr = [y;p;r];
        return
    end
end