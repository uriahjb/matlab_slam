%{
    Transform scan data from local to world
    coordinates
%}
function lidar_points = tform_scan( state, ranges, lidar_norm )
        tf = tforms(state);
        lidar_dat = [[ranges ranges].*lidar_norm' ones(length(ranges),1)];
        ranges = [ranges zeros(length(ranges),1)];
        %lidar_dat_old = cell2mat(arrayfun( @(j) [lidar_rots(:,:,j)*ranges(j,:)'; 1], 1:length(ranges), 'UniformOutput', false ));
        lidar_points = tf*lidar_dat';
end
