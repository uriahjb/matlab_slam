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

p_hit = 0.65;
p_miss = 1 - p_hit;
cfg.log_hit = log(p_hit/p_miss);
cfg.confidence_thresh = log(0.999/0.001);

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
world_size = 30;

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



ts0 = lidar.ts(1);
tic()
for ind = 1000:length(lidar.ts)-100

    if lidar.ts(ind) - ts0 > 1.0
        disp(['logtime: ' num2str(lidar.ts(ind)) ' realtime: ' num2str(toc())]);
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
    
    if mod(ind, 20) == 0

        % Perform local theta search 
        
        
        % convert map to int8 
        map_int8 = int8(256/(2*cfg.confidence_thresh).*map);
        %x_im = -map_center(1):resolution:map_size(1)-map_center(1);
        %y_im = -map_center(2):resolution:map_size(2)-map_center(2);

        x_imap = (0:resolution:map_size(1)*resolution) - map_center(1)*resolution;
        y_imap = (0:resolution:map_size(2)*resolution) - map_center(2)*resolution;
        search_range = 0.1;
        x_range = state(1)-search_range:resolution:state(1)+search_range;
        y_range = state(2)-search_range:resolution:state(2)+search_range;

        lidar_corr = [lidar_hits(1,:); lidar_hits(2,:); zeros(1,num_scans)];

        c = map_correlation(map_int8, x_imap,y_imap ...
                            ,lidar_corr ...
                            ,x_range,y_range);
        [vs,ixv] = max(c);
        [v, iy] = max(vs);
        ix = ixv(iy);

        ix
        iy

        if v ~= mean(mean(c))
            dpos = [(length(x_range)/2-ix)*resolution, (length(y_range)/2-iy)*resolution ];
        else
            dpos = [0 0];
        end
    else
        dpos = [0 0];
    end
    
    
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
    ray_inds = setdiff( ray_inds, hit_inds );
    
    % Subtract miss probabilities
    map(ray_inds) = max(min(map(ray_inds)-cfg.log_hit, cfg.confidence_thresh) ...
                       , -cfg.confidence_thresh); 
    
    % Add hit probabilities
    % Bound map values by max-min confidence thresholds
    map(hit_inds) = max(min(map(hit_inds)+cfg.log_hit, cfg.confidence_thresh) ...
                       , -cfg.confidence_thresh); 
                        
    %% Update state                  
    [vel, w, dt] = get_vel( ind, dat, cfg );
    alpha = 0.25;
    state(1:2) = state(1:2) - alpha*dpos;
    %state(1:2) = state(1:2) - dpos;
    state = update_motion( state, vel, w, dt );
    %toc
    
    if mod(ind, 1) == 0
        set(im,'CData', scl*(map+cfg.confidence_thresh)) 
        drawnow
    end
end
    
    
    
    
    
    

    


