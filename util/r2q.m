function q = r2q(r)

    % lets see how many errors show up here :)
    q = zeros(4,1);

    debug = false;
    
    % Select four cases based on the magnitude of the square root
    t0 = trace(r) + 1.0;
    t1 = r(1,1) - r(2,2) - r(3,3) + 1.0;
    t2 = -r(1,1) + r(2,2) - r(3,3) + 1.0;
    t3 = -r(1,1) - r(2,2) + r(3,3) + 1.0;
    
    [t,case_num] = max([t0 t1 t2 t3]); 
    if ( case_num == 1)
        if ( debug )
            disp('case 1');
        end
        q(1) = 0.5*sqrt(t);
        s = 4.0*(t/4.0)^0.5;
        q(2) = (r(3,2)-r(2,3))/s;
        q(3) = (r(1,3)-r(3,1))/s;
        q(4) = (r(2,1)-r(1,2))/s;
        
    elseif ( case_num == 2 )
        if ( debug )
            disp('case 2');
        end
        q(2) = 0.5*sqrt(t);
        s = 4.0*(t/4.0)^0.5;
        q(1) = (r(3,2)-r(2,3))/s;
        q(3) = (r(1,2)+r(2,1))/s;
        q(4) = (r(1,3)+r(3,1))/s;
                
    elseif ( case_num == 3 )
        if ( debug )
            disp('case 3');
        end
        s = 4.0*(t/4.0)^0.5;
        q(3) = 0.5*sqrt(t);
        q(1) = (r(1,3)-r(3,1))/s;
        q(2) = (r(1,2)+r(2,1))/s;
        q(4) = (r(2,3)+r(3,2))/s;
        
    elseif ( case_num == 4 )
        if ( debug ) 
            disp('case 4');
        end
        t = -r(1,1)-r(2,2)+r(3,3) + 1.0;
        s = 4.0*(t/4.0)^0.5;
        q(4) = 0.5*sqrt(t);
        q(1) = (r(2,1)-r(1,2))/s;
        q(2) = (r(1,3)+r(3,1))/s;
        q(3) = (r(2,3)+r(3,2))/s;
    else
        disp(' something went wrong :( ');
    end
       
end
