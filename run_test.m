function res = run_test( test_num )

    %% Load in data
    dat = load_measurements( test_num );

    %% World Configuration
    world.resolution = 0.05;
    world.width = 75;
    world.size = [world.width world.width]./world.resolution;
    world.center = world.size./2;
    world.map = ones(world.size)*log(0.5/0.5); % initialize map as unknown

    %% Robot Configuration
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
    cfg.unknown = log(0.5/0.5);

    cfg.search_range = world.resolution*2;
    cfg.theta_range = 0.25;
    cfg.num_thetas = 25;

    %% For plotting
    vec = [0 0 1; 0 1 1; 0 0 1; 1 0 1]';

    lidar = dat.hokuyo;
    lidar_rots = rot(lidar.angles);
    num_scans = length(lidar.angles);

    ranges_norm = [ones(length(lidar.angles),1) zeros(length(lidar.angles),1)];
    lidar.norm = cell2mat(arrayfun( @(j) lidar_rots(:,:,j)*ranges_norm(j,:)', 1:length(lidar.angles), 'UniformOutput', false ));

    %% For drawing trajectory, current position and map
    figure(1)

    range = [0 1.0];
    scl = (range(2)-range(1))/(cfg.confidence_thresh+cfg.confidence_thresh);
    im = imshow(scl*(world.map+cfg.confidence_thresh), 'Border', 'tight');
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
    trajectory = [];


    %% Init
    state = [0 0 0];
    ts0 = lidar.ts(1);
    tic()

    for ind = 1:length(lidar.ts)-100
        % Display current time
        if lidar.ts(ind) - ts0 > 1.0
            disp(['logtime: ' num2str(lidar.ts(ind)) ' realtime: ' num2str(toc()) ' ind: ' num2str(ind)]);
            ts0 = lidar.ts(ind);
        end

        %% Scan match and find most likely new state
        state = scan_match( ind, state, lidar, world, cfg );    

        %% Given state and lidar hits update the map
        world = update_map( ind, state, lidar, world, cfg );    

        %% Save trajectory and draw
        state_inds = double(int32(1/world.resolution*state(1:2)) + int32(world.center));

        % Draw current state
        set(ph, 'XData', state_inds(2), 'YData', state_inds(1) ); 

        % Draw trajectory
        traj = [traj; state_inds];   
        set(trajh, 'XData', traj(:,2), 'YData', traj(:,1) );

        trajectory = [trajectory; state];

        %% Update state                  
        [vel, w, dt] = get_vel( ind, dat, cfg );

        state = update_motion( state, vel, w, dt );

        if mod(ind, 25) == 0
            set(im,'CData', scl*(world.map+cfg.confidence_thresh))         
            drawnow
        end    
    end

    res.traj = trajectory;
    res.world = world;
    res.cfg = cfg;
    res.lidar = lidar;
    res.t0 = dat.t0;

end
    
    
    
    
    

    


