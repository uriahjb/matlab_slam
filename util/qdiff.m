%{
    Return the difference of two quaternions
%}
function q_diff = qdiff( q0, q1 )
    q_diff = cell2mat( arrayfun( @(j) ( qmult( qconj(q0(:,j)), q1(:,j) ) ), 1:size(q0,2), 'UniformOutput', false ) );
end