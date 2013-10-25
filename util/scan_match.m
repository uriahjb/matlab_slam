function new_state = scan_match( ind, state, lidar, world, cfg )
    res = world.resolution;    
    map_center = world.center;
    map_size = world.size;        
    search_range = cfg.search_range;
    num_thetas = cfg.num_thetas;   
    num_scans = length(lidar.angles);
    
    dthetas = rand(1,num_thetas)*cfg.theta_range-cfg.theta_range/2;
    
    % convert map to int8 
    map_int8 = int8(256/(2*cfg.confidence_thresh).*world.map);

    x_imap = (0:res:map_size(1)*res) - map_center(1)*res;
    y_imap = (0:res:map_size(2)*res) - map_center(2)*res;
    x_range = state(1)-search_range:res:state(1)+search_range;
    y_range = state(2)-search_range:res:state(2)+search_range;

    vals = [];
    dstates = [];
    for th = 1:length(dthetas)
        
        state_search(3) = state(3)+dthetas(th);

        lidar_hits_search = tform_scan( state_search, lidar.ranges(:,ind), lidar.norm);
        lidar_corr = [lidar_hits_search(1,:); lidar_hits_search(2,:); zeros(1,num_scans)];

        c = map_correlation(map_int8, x_imap,y_imap ...
                            ,lidar_corr ...
                            ,x_range,y_range);
        [vs,ixv] = max(c);
        [v, iy] = max(vs);
        ix = ixv(iy);

        vals = [vals v];

        if v ~= mean(mean(c))
            dstates = [dstates; [(length(x_range)/2-ix)*res, (length(y_range)/2-iy)*res dthetas(th)] ];
        else
            dstates = [dstates; [0 0 0]];
        end
    end

    % Take most likely state and provide and update
    [mv,mi] = max(vals);
    dpos = dstates(mi,:);    
    new_state(1:2) = state(1:2) - dpos(1:2);    
    new_state(3) = state(3) + dpos(3);
end
    