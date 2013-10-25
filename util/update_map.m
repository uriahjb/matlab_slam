%{
    Update map given new state and lidar hits
%}
function world = update_map( ind, state, lidar, world, cfg )
    
    lidar_hits = tform_scan( state, lidar.ranges(:,ind), lidar.norm );
    
    state_inds = double(int32(1/world.resolution*state(1:2)) + int32(world.center));
        
    lidar_cell_hits = bsxfun( @plus, int32(1/world.resolution*lidar_hits(1:2,:)), int32(world.center') );
    
    % Caculate inds where lidar rays passed through.
    [rays_ix,rays_iy] = getMapCellsFromRay(state_inds(1),state_inds(2) ...
                                           , double(lidar_cell_hits(1,:)),double(lidar_cell_hits(2,:)));
    
    %% Apply probabilities to map
    
    hit_inds = sub2ind( world.size, lidar_cell_hits(1,:), lidar_cell_hits(2,:) );
    ray_inds = sub2ind( world.size, rays_ix, rays_iy );

    % Find unique ray_inds that are not in hit_inds
    %ray_inds = setdiff( ray_inds, hit_inds );
    
    
    % Here I should try to convert to an int8 map instead of everywhere else
    %map_int8 = int8(256/(2*cfg.confidence_thresh).*map);
    % Subtract miss probabilities
    world.map(ray_inds) = max( min( world.map(ray_inds)-cfg.log_hit, cfg.confidence_thresh ) ...
                             , -cfg.confidence_thresh); 
    
    % Add hit probabilities
    % Bound map values by max-min confidence thresholds
    world.map(hit_inds) = max(min(world.map(hit_inds)+2.0*cfg.log_hit, cfg.confidence_thresh) ...
                       , -cfg.confidence_thresh); 
end
    