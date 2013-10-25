%{ 
    return rotation matrices of length theta
%}

function r = rot( thetas )
    
    r = reshape( cell2mat( arrayfun( @(t) [cos(thetas(t)) -sin(thetas(t)); sin(thetas(t)) cos(thetas(t))], ...
                  1:length(thetas), 'UniformOutput', false ) ...
                  ), 2, 2, length(thetas) );
    
    % This trick is way faster
    %r = reshape( [cos(thetas); sin(thetas); -sin(thetas); cos(thetas)], 2, 2, length(thetas) );
end