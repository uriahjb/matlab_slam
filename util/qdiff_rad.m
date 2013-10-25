%{
    Return difference between two quaternions in radians
%}
function q_diff_rad = qdiff_rad( q0, q1 )
    q_diff = qdiff( q0, q1 );
    q_diff_rad = arrayfun( @(j) ( 2*acos(q_diff(1,j)) ), 1:size(q_diff,2) );
    q_diff_rad( q_diff_rad > pi ) = abs( q_diff_rad( q_diff_rad > pi ) - 2*pi );
    q_diff_rad = q_diff_rad( isnan(q_diff_rad) == 0 );
end