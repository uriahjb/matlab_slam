%{ 
    Just using encoder data produce a map
%}

%% Load in data
dat = load_measurements(20);

%% Configuration Parameters
wheel_circ = 2*pi*0.165;
cnt_to_rad = 1/360;

cfg = struct();
cfg.imu_scl = 1050/1023*pi/180;
cfg.imu_bias = 370;
cfg.cnt_to_vel = 8.0*wheel_circ*cnt_to_rad;

p_hit = 0.51;
p_miss = 1 - p_hit;
cfg.log_hit = log(p_hit/p_miss);
cfg.confidence_thresh = log(0.99999/0.00001);

%% For plotting
vec = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

lidar = dat.hokuyo;
lidar_rots = rot(lidar.angles);
num_scans = length(lidar.angles);

ranges_norm = [ones(length(lidar.angles),1) zeros(length(lidar.angles),1)];
lidar_norm = cell2mat(arrayfun( @(j) lidar_rots(:,:,j)*ranges_norm(j,:)', 1:length(lidar.angles), 'UniformOutput', false ));

%% Init
state = [0 0 0];

resolution = 0.05;
world_size = 75;

map_size = [world_size world_size]./resolution; % width in cells
map_center = map_size./2;
%world_size = 10; % meters
unknown = log(0.5/0.5);
map = ones(map_size)*unknown; % initialize map with uncertain

%% Want to draw particles 
figure(1)

range = [0 1.0];
scl = (range(2)-range(1))/(cfg.confidence_thresh+cfg.confidence_thresh);
im = imshow(scl*(map+cfg.confidence_thresh), 'Border', 'tight');
set(im, 'clipping', 'off');

ax = get(gcf, 'CurrentAxes');
set(ax, 'YLimMode', 'auto');
set(ax, 'XLimMode', 'auto');
set(ax, 'PlotBoxAspectRatioMode', 'auto');


ph = line();
set(ph, 'XData', [], 'YData', []);
set(ph, 'LineStyle', 'None');
set(ph, 'Marker', 's');
set(ph, 'Color', 'r' );

trajh = line();
set(trajh, 'XData', [], 'YData', []);
set(trajh, 'Color', 'r' );
traj = int32([]);

search_range = resolution*3;
theta_range = 0.1;
num_thetas = 15;

ts0 = lidar.ts(1);
tic()
start_ind = 500;
end_ind = 100;
for ind = start_ind:length(lidar.ts)-end_ind;

    if lidar.ts(ind) - ts0 > 1.0
        disp(['logtime: ' num2str(lidar.ts(ind)) ' realtime: ' num2str(toc()) ' ind: ' num2str(ind)]);
        ts0 = lidar.ts(ind);
    end
    
    %tic
    
    %% Transform lidar hits from local frame to map coordinates
    lidar_hits = tform_scan( state, lidar.ranges(:,ind), lidar_norm);

    
    %state_inds = double(int32(1/resolution*state(1:2) + map_center(1)));
    %state_inds = double(int32(map_size(1)/world_size*state(1:2) + map_center(1)));
    state_inds = double(int32(1/resolution*state(1:2)) + int32(map_center));
    
    
    % Draw current state
    set(ph, 'XData', state_inds(2), 'YData', state_inds(1) ); 

    % Draw trajectory
    traj = [traj; state_inds];
    set(trajh, 'XData', traj(:,2), 'YData', traj(:,1) );
    
    %lidar_cell_hits = int32(1/resolution*lidar_hits(1:2,:))+map_center(1);
    %lidar_cell_hits = int32(map_size(1)/world_size*lidar_hits(1:2,:))+map_center(1);
    lidar_cell_hits = bsxfun( @plus, int32(1/resolution*lidar_hits(1:2,:)), int32(map_center') );

    
    % Caculate inds where lidar rays passed through.
    [rays_ix,rays_iy] = getMapCellsFromRay(state_inds(1),state_inds(2) ...
                                           , double(lidar_cell_hits(1,:)),double(lidar_cell_hits(2,:)));
   
    %% Resize Map 
    % Need to think of a good way to resize the map whenever the current
    % bounds have been reached
    
    rays_xy = [rays_ix rays_iy];
    out_of_bounds_x = [lidar_cell_hits(1, lidar_cell_hits(1,:) > map_size(1) )'; ...
                       lidar_cell_hits(1, lidar_cell_hits(1,:) < 1 )'; ...
                       rays_ix( rays_ix > map_size(1) ); ...
                       rays_ix( rays_ix < 1 )];
    out_of_bounds_y = [lidar_cell_hits(2, lidar_cell_hits(2,:) > map_size(2) )'; ...
                       lidar_cell_hits(2, lidar_cell_hits(2,:) < 1 )'; ...
                       rays_iy( rays_iy > map_size(2) ); ...
                       rays_iy( rays_iy < 1 )];
    % Deal with x axis first
    padding = 10;
    if ~isempty(out_of_bounds_x)
        min_x = min(min(out_of_bounds_x), 1);
        max_x = max(max(out_of_bounds_x), map_size(1));
        map = [ones(1-min_x, map_size(2)).*unknown; ...
               map; ...
               ones(max_x-map_size(1),map_size(2)).*unknown];
        disp(['resize x: ' num2str(min_x-1) ', ' num2str(max_x-map_size(1))]); 
        map_size = size(map);
        if min_x < 1
            dcenter = abs(min_x)+1
            map_center(1) = map_center(1)+dcenter;
            lidar_cell_hits(1,:) = lidar_cell_hits(1,:)+dcenter;
            rays_ix = rays_ix+double(dcenter);
        end
        set(im,'CData', scl*(map+cfg.confidence_thresh)) 
        drawnow
    end
    % Then handle y axis
    if ~isempty(out_of_bounds_y)
        min_y = min(min(out_of_bounds_y), 1);
        max_y = max(max(out_of_bounds_y), map_size(2));
        size(map)
        map = [ones(map_size(1),1-min_y).*unknown ...
               map ...
               ones(map_size(1),max_y-map_size(2)).*unknown];
        disp(['resize y: ' num2str(1-min_y) ', ' num2str(max_y-map_size(2))]);
        map_size = size(map)
        if min_y < 1
            dcenter = abs(min_y)+1
            map_center(2) = map_center(2)+dcenter;
            lidar_cell_hits(2,:) = lidar_cell_hits(2,:)+dcenter;
            rays_iy = rays_iy+double(dcenter);
        end
        set(im,'CData', scl*(map+cfg.confidence_thresh)) 
        drawnow
    end 
   
     %% Compute map correlations occasionally
    
    if mod(ind, 1) == 0

        % Perform local theta search 
        %dthetas = -0.05:0.01:0.05;
        

        dthetas = rand(1,num_thetas)*theta_range-theta_range/2;
        vals = [];
        dstates = [];
        

        % convert map to int8 
        map_int8 = int8(256/(2*cfg.confidence_thresh).*map);
        
        for th = 1:length(dthetas)

            %x_im = -map_center(1):resolution:map_size(1)-map_center(1);
            %y_im = -map_center(2):resolution:map_size(2)-map_center(2);

            x_imap = (0:resolution:map_size(1)*resolution) - map_center(1)*resolution;
            y_imap = (0:resolution:map_size(2)*resolution) - map_center(2)*resolution;
            x_range = state(1)-search_range:resolution:state(1)+search_range;
            y_range = state(2)-search_range:resolution:state(2)+search_range;

            state_search(3) = state(3)+dthetas(th);
       
            lidar_hits_search = tform_scan( state_search, lidar.ranges(:,ind), lidar_norm);
            lidar_corr = [lidar_hits_search(1,:); lidar_hits_search(2,:); zeros(1,num_scans)];

            c = map_correlation(map_int8, x_imap,y_imap ...
                                ,lidar_corr ...
                                ,x_range,y_range);
            [vs,ixv] = max(c);
            [v, iy] = max(vs);
            ix = ixv(iy);
                        
            vals = [vals v];

            if v ~= mean(mean(c))
                dstates = [dstates; [(length(x_range)/2-ix)*resolution, (length(y_range)/2-iy)*resolution dthetas(th)] ];
            else
                dstates = [dstates; [0 0 0]];
            end
        end
        [mv,mi] = max(vals);
        dpos = dstates(mi,:);
    else
        dpos = [0 0 0];
    end
    
    alpha = 1.0;
    state(1:2) = state(1:2) - alpha*dpos(1:2);
    beta = 1.0;
    state(3) = state(3) + beta*dpos(3);
    
    lidar_hits = tform_scan( state, lidar.ranges(:,ind), lidar_norm);
    
    %state_inds = double(int32(1/resolution*state(1:2) + map_center(1)));
    %state_inds = double(int32(map_size(1)/world_size*state(1:2) + map_center(1)));
    state_inds = double(int32(1/resolution*state(1:2)) + int32(map_center));
        
    % Draw current state
    set(ph, 'XData', state_inds(2), 'YData', state_inds(1) ); 

    % Draw trajectory
    traj = [traj; state_inds];
    set(trajh, 'XData', traj(:,2), 'YData', traj(:,1) );
    
    %lidar_cell_hits = int32(1/resolution*lidar_hits(1:2,:))+map_center(1);
    %lidar_cell_hits = int32(map_size(1)/world_size*lidar_hits(1:2,:))+map_center(1);
    lidar_cell_hits = bsxfun( @plus, int32(1/resolution*lidar_hits(1:2,:)), int32(map_center') );

    
    % Caculate inds where lidar rays passed through.
    [rays_ix,rays_iy] = getMapCellsFromRay(state_inds(1),state_inds(2) ...
                                           , double(lidar_cell_hits(1,:)),double(lidar_cell_hits(2,:)));
    
    %plot original lidar points
    %{
    figure(3);
    lidar_cell_hits = bsxfun( @plus, int32(1/resolution*lidar_hits(1:2,:)), int32(map_center') );

  
    %plot map
    imagesc(map_int8);
    hold on;
    plot(lidar_cell_hits(2,:),lidar_cell_hits(1,:),'r.')
    
    %plot correlation
    figure(4);
    surf(c)
    
    input('');
    %}
        
    %% Apply probabilities to map
    
    hit_inds = sub2ind( map_size, lidar_cell_hits(1,:), lidar_cell_hits(2,:) );
    ray_inds = sub2ind( map_size, rays_ix, rays_iy );

    % Find unique ray_inds that are not in hit_inds
    %ray_inds = setdiff( ray_inds, hit_inds );
    
    % Subtract miss probabilities
    map(ray_inds) = max(min(map(ray_inds)-cfg.log_hit, cfg.confidence_thresh) ...
                       , -cfg.confidence_thresh); 
    
    % Add hit probabilities
    % Bound map values by max-min confidence thresholds
    map(hit_inds) = max(min(map(hit_inds)+2.0*cfg.log_hit, cfg.confidence_thresh) ...
                       , -cfg.confidence_thresh); 
                        
    %% Update state                  
    [vel, w, dt] = get_vel( ind, dat, cfg );
    
    %state(1:2) = state(1:2) - dpos;
    state = update_motion( state, vel, w, dt );
    %toc
    
    if mod(ind, 10) == 0
        set(im,'CData', scl*(map+cfg.confidence_thresh))         
    end
    drawnow
end

test_dat = struct();
test_dat.map = scl*(map+cfg.confidence_thresh);
test_dat.traj = traj;
test_dat.start_ind = start_ind;
test_dat.end_ind = ind;
    
    
    
    

    


