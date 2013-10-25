%{
    calc tforms from states, just bc its useful
%}

function r = tforms( states )
    r = reshape( cell2mat( arrayfun( @(t) [cos(states(t,3)) -sin(states(t,3)) states(t,1); sin(states(t,3)) cos(states(t,3)) states(t,2); 0 0 1], ...
                  1:size(states,1), 'UniformOutput', false ) ...
                  ), 3, 3, size(states,1) );
end
            